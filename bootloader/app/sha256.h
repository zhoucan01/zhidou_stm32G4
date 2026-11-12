#ifndef OTA_SHA256_H
#define OTA_SHA256_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t state[8];
    uint64_t total_bytes;
    uint8_t block[64];
    size_t used;
} sha256_ctx;

/* 初始化 SHA-256 上下文。 */
void sha256_init(sha256_ctx *ctx);
/* 分段追加原始数据；允许任意分段长度。 */
void sha256_update(sha256_ctx *ctx, const void *data, size_t size);
/* 输出 32 字节大端摘要；结束后重新使用需先初始化。 */
void sha256_final(sha256_ctx *ctx, uint8_t digest[32]);

#endif
