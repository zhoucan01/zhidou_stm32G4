"""持续 OTA 压力测试：只有 APP 回读匹配才成功，详细日志逐行落盘。"""
from datetime import datetime
from pathlib import Path
import json
import re
import secrets
import shutil
import threading
import time

from images import set_version, validate_ota
from serial_ota import validate_timeout

MIB=1024*1024
RUN_LOG_BUDGET=64*MIB
DIRECTORY_LOG_BUDGET=512*MIB
MIN_FREE_SPACE=256*MIB


class BoundedLog:
    """Only rotate this run's exclusively created files; never touch old runs."""
    def __init__(self,path,max_bytes,backups=3):
        self.path=Path(path);self.max_bytes=max_bytes;self.backups=backups;self.size=0
        self.stream=self.path.open('xb')
    def write(self,text):
        data=text.encode('utf-8')
        if len(data)>self.max_bytes:
            data=data[:self.max_bytes-64].decode('utf-8',errors='ignore').encode('utf-8')+b' [line truncated]\n'
        if self.size+len(data)>self.max_bytes:
            self.stream.close()
            oldest=Path(str(self.path)+'.'+str(self.backups))
            oldest.unlink(missing_ok=True)
            for number in range(self.backups-1,0,-1):
                source=Path(str(self.path)+'.'+str(number))
                if source.exists():source.replace(Path(str(self.path)+'.'+str(number+1)))
            self.path.replace(Path(str(self.path)+'.1'))
            self.stream=self.path.open('xb');self.size=0
        self.stream.write(data);self.size+=len(data)
    def flush(self):self.stream.flush()
    def close(self):self.stream.close()


def version_parts(version):
    if not re.fullmatch(r'[0-9]+(?:\.[0-9]+)*',version):
        raise ValueError('压力测试起始版本请填写数字版本，例如1.0.1或1.0.1.0')
    return tuple(int(part) for part in version.split('.'))


def next_version(seed,current=None):
    parts=version_parts(seed)
    if current:
        try:other=version_parts(current)
        except ValueError:other=()
        if len(other)==len(parts) and other>parts:parts=other
    return '.'.join(map(str,parts[:-1]+(parts[-1]+1,)))


class RunLog:
    def __init__(self,directory):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        existing=sum(p.stat().st_size for p in directory.glob('ota_stress_*')
                     if p.is_file() and re.fullmatch(r'ota_stress_\d{8}_\d{6}_\d{6}(?:\.commands)?\.(?:log|jsonl)(?:\.[1-3])?',p.name))
        if existing+RUN_LOG_BUDGET>DIRECTORY_LOG_BUDGET:
            raise OSError('日志目录已接近512MiB保留上限，请转移旧日志或选择其他目录；不会自动删除历史日志')
        if shutil.disk_usage(directory).free<MIN_FREE_SPACE:
            raise OSError('日志磁盘剩余空间不足256MiB，请释放空间或选择其他目录')
        self.directory=directory;self.next_space_check=time.monotonic()+1
        self.path=directory/('ota_stress_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.lock=threading.Lock();self.error=None;self.closed=False
        self.detail=BoundedLog(self.path.with_suffix('.log'),8*MIB)
        try:
            self.commands=BoundedLog(self.path.with_suffix('.commands.log'),4*MIB)
            try:self.summary=BoundedLog(self.path.with_suffix('.jsonl'),4*MIB)
            except Exception:self.commands.close();raise
        except Exception:self.detail.close();raise
    def write(self,text):
        with self.lock:
            if self.closed:return
            try:
                self._check_space()
                self.detail.write(datetime.now().isoformat(timespec='milliseconds')+' '+str(text)+'\n')
                self.detail.flush()
            except OSError as exc:self.error=exc;raise
    def write_command(self,text):
        with self.lock:
            if self.closed:return
            try:
                self._check_space()
                self.commands.write(datetime.now().isoformat(timespec='milliseconds')+' '+str(text)+'\n')
                self.commands.flush()
            except OSError as exc:self.error=exc;raise
    def record(self,value):
        with self.lock:
            self._check_space()
            self.summary.write(json.dumps(value,ensure_ascii=False)+'\n');self.summary.flush()
        self.check()
    def _check_space(self):
        if self.error:raise OSError('日志写入已停止：'+str(self.error))
        now=time.monotonic()
        if now>=self.next_space_check:
            self.next_space_check=now+1
            if shutil.disk_usage(self.directory).free<MIN_FREE_SPACE:
                self.error=OSError('日志磁盘剩余空间不足256MiB，已停止写入以保护磁盘空间')
                raise self.error
    def check(self):
        if self.error:raise OSError('压力测试日志写入失败：'+str(self.error))
    def close(self):
        with self.lock:
            self.closed=True;self.detail.close();self.commands.close();self.summary.close()


class StressTest:
    def __init__(self,ota):
        self.ota=ota;self.stop_event=threading.Event();self.running=False;self.last_version=None
    def stop(self):
        self.stop_event.set()
        # The GUI calls this directly: do not acquire the disk logger lock here.
        self.ota.event('activity','请求停止压力测试：完成当前轮后停止')
    def read_current(self,timeout):
        try:
            return self.ota.query_version(timeout)
        except (ValueError,TimeoutError) as exc:
            # Erasure or an interrupted first packet can leave no decodable
            # version. Require a positive Bootloader status before recovery.
            state=self.ota.query(timeout)
            if state['status'] not in (1,2,0x55):
                raise RuntimeError('版本读取失败，且设备未确认处于可升级 Bootloader 状态') from exc
            self.ota.log('版本区无有效应答，已确认 Bootloader 状态 0x%04X；重新完整升级。原因：%s'%
                         (state['status'],exc))
            self.ota.command('恢复升级：Bootloader 已确认，版本未知，从首包重新开始')
            return {'mode':'bootloader','version':None,'recovery':True,'version_error':str(exc)}
    def wait_app(self,expected,startup_timeout):
        deadline=time.monotonic()+startup_timeout;last=None
        while time.monotonic()<deadline:
            if self.ota.stop_event.is_set():raise RuntimeError('串口已关闭')
            try:
                last=self.ota.query_version(timeout=min(1, max(.001,deadline-time.monotonic())))
                self.last_version=last
                if last['mode']=='app':
                    if last['version']!=expected:
                        raise ValueError('APP版本不匹配：期望%s，实际%s'%(expected,last['version']))
                    return last
            except TimeoutError:pass
            self.ota.stop_event.wait(min(.2,max(0,deadline-time.monotonic())))
        raise TimeoutError('等待APP启动超时；最后版本应答=%r'%last)
    def run(self,image,seed,timeout,startup_timeout,log_directory,firmware_path='',payload=1024):
        validate_ota(image);version_parts(seed)
        timeout=validate_timeout(timeout);startup_timeout=validate_timeout(startup_timeout)
        if not isinstance(payload,int) or not 1<=payload<=1024:
            raise ValueError('每包数据必须为1～1024字节')
        if (len(image)+payload-1)//payload>65536:
            raise ValueError('分包数量超出协议限制，请增大每包数据字节数')
        self.running=True
        counts={'rounds':0,'success':0,'failed':0};version=seed
        previous_sink=self.ota.log_sink;previous_commands=self.ota.command_sink
        try:
            with_log=RunLog(log_directory)
            self.ota.log_sink=with_log.write
            self.ota.command_sink=with_log.write_command
            try:
                self.ota.log('压力测试启动；固件=%s；详细日志=%s.log；轮次记录=%s.jsonl'%(firmware_path,with_log.path,with_log.path))
                self.ota.log('简明指令日志：%s.commands.log'%with_log.path)
                self.ota.command('日志按大小循环保留：本次最多64MiB；超过后覆盖本次最旧分卷，历史测试文件不会自动删除')
                self.ota.command('开始压力测试：固件=%s，起始版本=%s'%(firmware_path,seed))
                self.ota.log('配置：起始版本=%s 应答超时=%ss APP启动超时=%ss 分包=%d 首包=%d'%(seed,timeout,startup_timeout,payload,0))
                while not self.stop_event.is_set() and not self.ota.stop_event.is_set():
                    with_log.check()
                    counts['rounds']+=1;started=time.monotonic();stage='读取当前版本'
                    self.ota.command('第%d轮开始'%counts['rounds'])
                    record={'round':counts['rounds'],'started':datetime.now().isoformat(timespec='milliseconds'),
                            'firmware':firmware_path,'expected':None,'actual':None,'result':'failed'}
                    try:
                        self.last_version=None
                        current=self.read_current(timeout)
                        record['recovery']=current.get('recovery',False)
                        if current.get('version_error'):record['version_error']=current['version_error']
                        version=next_version(version,current['version']);record['expected']=version
                        packed,_=set_version(image,version)
                        self.ota.log('========== 压测第%d轮，目标版本 %s =========='%(counts['rounds'],version))
                        self.ota.command('本轮目标版本：'+version)
                        key=secrets.randbits(32)
                        stage='进入Bootloader'
                        if current['mode']=='app':self.ota.reset_and_query(key,timeout)
                        stage='升级固件';self.ota.upgrade(packed,key,timeout,payload=payload)
                        stage='等待APP并核对版本';actual=self.wait_app(version,startup_timeout)
                        record['actual']=actual;record['result']='success';counts['success']+=1
                        self.ota.log('本轮成功：APP已运行，版本完全匹配 '+version)
                        self.ota.command('第%d轮成功：APP 已运行，版本匹配 %s'%(counts['rounds'],version))
                    except Exception as exc:
                        counts['failed']+=1;record['error']=str(exc)
                        record['actual']=self.last_version
                        self.ota.log('本轮失败：阶段=%s，原因=%s；下一轮重新握手，不重传旧数据包'%(stage,exc))
                        self.ota.command('第%d轮失败：%s；%s'%(counts['rounds'],stage,exc))
                    record.update(stage=stage,elapsed=round(time.monotonic()-started,3),**counts)
                    with_log.record(record)
                    self.ota.event('stress',dict(counts,version=version,log=str(with_log.path),result=record['result']))
                    self.ota.log('累计：%d轮，成功%d，失败%d'%(counts['rounds'],counts['success'],counts['failed']))
                    if self.stop_event.wait(1):break
                if self.stop_event.is_set():
                    self.ota.command('操作：用户请求停止，当前轮已结束')
                self.ota.log('压力测试停止：'+str(counts))
                self.ota.command('压力测试停止：共%d轮，成功%d，失败%d'%(counts['rounds'],counts['success'],counts['failed']))
                with_log.record({'event':'stopped','time':datetime.now().isoformat(),**counts})
            finally:
                self.ota.log_sink=previous_sink;self.ota.command_sink=previous_commands;with_log.close()
        finally:
            self.running=False
