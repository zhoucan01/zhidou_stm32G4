#include "sha256.h"
#include <string.h>

/* SHA-256 标准轮常量；使用循环和 16 字循环消息表节省代码与栈。 */
static const uint32_t k[64] = {
    0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,
    0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
    0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,
    0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
    0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,
    0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
    0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,
    0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
    0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,
    0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
    0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,
    0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
    0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,
    0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
    0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,
    0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U
};

static uint32_t ror(uint32_t x, unsigned n)
{
    return (x >> n) | (x << (32U - n));
}

/* 处理一个完整的 64 字节数据块。 */
static void sha256_block(sha256_ctx *ctx)
{
    uint32_t w[16], v[8], t1, t2, x, y;
    unsigned i, j;
    memcpy(v, ctx->state, sizeof(v));
    for(i = 0; i < 64; ++i)
    {
        j = i & 15U;
        if(i < 16)
        {
            const uint8_t *p = ctx->block + i * 4U;
            w[j] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                 | ((uint32_t)p[2] << 8) | p[3];
        }
        else
        {
            x = w[(i - 15U) & 15U];
            y = w[(i - 2U) & 15U];
            w[j] += (ror(x,7) ^ ror(x,18) ^ (x >> 3))
                  + w[(i - 7U) & 15U]
                  + (ror(y,17) ^ ror(y,19) ^ (y >> 10));
        }
        t1 = v[7] + (ror(v[4],6) ^ ror(v[4],11) ^ ror(v[4],25))
           + ((v[4] & v[5]) ^ (~v[4] & v[6])) + k[i] + w[j];
        t2 = (ror(v[0],2) ^ ror(v[0],13) ^ ror(v[0],22))
           + ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));
        for(j = 7; j > 0; --j)
            v[j] = v[j - 1];
        v[4] += t1;
        v[0] = t1 + t2;
    }
    for(i = 0; i < 8; ++i)
        ctx->state[i] += v[i];
}

void sha256_init(sha256_ctx *ctx)
{
    static const uint32_t initial[8] = {
        0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
        0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U
    };
    memcpy(ctx->state, initial, sizeof(initial));
    ctx->total_bytes = 0;
    ctx->used = 0;
}

void sha256_update(sha256_ctx *ctx, const void *data, size_t size)
{
    const uint8_t *p = (const uint8_t *)data;
    ctx->total_bytes += (uint64_t)size;
    while(size != 0)
    {
        size_t n = 64U - ctx->used;
        if(n > size)
            n = size;
        memcpy(ctx->block + ctx->used, p, n);
        ctx->used += n;
        p += n;
        size -= n;
        if(ctx->used == 64U)
        {
            sha256_block(ctx);
            ctx->used = 0;
        }
    }
}

void sha256_final(sha256_ctx *ctx, uint8_t digest[32])
{
    uint64_t bits = ctx->total_bytes << 3;
    unsigned i;
    ctx->block[ctx->used++] = 0x80;
    if(ctx->used > 56U)
    {
        memset(ctx->block + ctx->used, 0, 64U - ctx->used);
        sha256_block(ctx);
        ctx->used = 0;
    }
    memset(ctx->block + ctx->used, 0, 56U - ctx->used);
    for(i = 0; i < 8; ++i)
        ctx->block[63U - i] = (uint8_t)(bits >> (8U * i));
    sha256_block(ctx);
    for(i = 0; i < 32; ++i)
        digest[i] = (uint8_t)(ctx->state[i / 4U] >> (24U - 8U * (i & 3U)));
}
