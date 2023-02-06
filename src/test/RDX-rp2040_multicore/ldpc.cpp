//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
// Common configuration resources and definitions
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
#include <Arduino.h>
#include "RDX-rp2040.h"
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * ldpc.cpp 
 * LDPC decoder for ft8 decoding functions
 * Code excerpts from
 * originally from ft8_lib by Karlis Goba (YL3JG)
 * excerpts taken from pi_ft8_xcvr by Godwin Duan (AA1GD) 2021
 * excerpts taken from Orange_Thunder by Pedro Colla (LU7DZ) 2018
 *
 * Adaptation to ADX-rp2040 project by Pedro Colla (LU7DZ) 2022
 * 
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
//
// LDPC decoder for FT8.
//
// given a 174-bit codeword as an array of log-likelihood of zero,
// return a 174-bit corrected codeword, or zero-length array.
// last 87 bits are the (systematic) plain-text.
// this is an implementation of the sum-product algorithm
// from Sarah Johnson's Iterative Error Correction book.
// codeword[i] = log ( P(x=0) / P(x=1) )
//

#include "ldpc.h"
#include "constants.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

static int ldpc_check(uint8_t codeword[]);
static float fast_tanh(float x);
static float fast_atanh(float x);

// Packs a string of bits each represented as a zero/non-zero byte in plain[],
// as a string of packed bits starting from the MSB of the first byte of packed[]
void pack_bits(const uint8_t plain[], int num_bits, uint8_t packed[])
{
    int num_bytes = (num_bits + 7) / 8;
    for (int i = 0; i < num_bytes; ++i)
    {
        packed[i] = 0;
    }

    uint8_t mask = 0x80;
    int byte_idx = 0;
    for (int i = 0; i < num_bits; ++i)
    {
        if (plain[i])
        {
            packed[byte_idx] |= mask;
        }
        mask >>= 1;
        if (!mask)
        {
            mask = 0x80;
            ++byte_idx;
        }
    }
}

// codeword is 174 log-likelihoods.
// plain is a return value, 174 ints, to be 0 or 1.
// max_iters is how hard to try.
// ok == 87 means success.
void ldpc_decode(float codeword[], int max_iters, uint8_t plain[], int *ok)
{
    float m[FT8_LDPC_M][FT8_LDPC_N]; // ~60 kB
    float e[FT8_LDPC_M][FT8_LDPC_N]; // ~60 kB
    int min_errors = FT8_LDPC_M;

    for (int j = 0; j < FT8_LDPC_M; j++)
    {
        for (int i = 0; i < FT8_LDPC_N; i++)
        {
            m[j][i] = codeword[i];
            e[j][i] = 0.0f;
        }
    }

    for (int iter = 0; iter < max_iters; iter++)
    {
        for (int j = 0; j < FT8_LDPC_M; j++)
        {
            for (int ii1 = 0; ii1 < kFT8_LDPC_num_rows[j]; ii1++)
            {
                int i1 = kFT8_LDPC_Nm[j][ii1] - 1;
                float a = 1.0f;
                for (int ii2 = 0; ii2 < kFT8_LDPC_num_rows[j]; ii2++)
                {
                    int i2 = kFT8_LDPC_Nm[j][ii2] - 1;
                    if (i2 != i1)
                    {
                        a *= fast_tanh(-m[j][i2] / 2.0f);
                    }
                }
                e[j][i1] = -2.0f * fast_atanh(a);
            }
        }

        for (int i = 0; i < FT8_LDPC_N; i++)
        {
            float l = codeword[i];
            for (int j = 0; j < 3; j++)
                l += e[kFT8_LDPC_Mn[i][j] - 1][i];
            plain[i] = (l > 0) ? 1 : 0;
        }

        int errors = ldpc_check(plain);

        if (errors < min_errors)
        {
            // Update the current best result
            min_errors = errors;

            if (errors == 0)
            {
                break; // Found a perfect answer
            }
        }

        for (int i = 0; i < FT8_LDPC_N; i++)
        {
            for (int ji1 = 0; ji1 < 3; ji1++)
            {
                int j1 = kFT8_LDPC_Mn[i][ji1] - 1;
                float l = codeword[i];
                for (int ji2 = 0; ji2 < 3; ji2++)
                {
                    if (ji1 != ji2)
                    {
                        int j2 = kFT8_LDPC_Mn[i][ji2] - 1;
                        l += e[j2][i];
                    }
                }
                m[j1][i] = l;
            }
        }
    }

    *ok = min_errors;
}

//
// does a 174-bit codeword pass the FT8's LDPC parity checks?
// returns the number of parity errors.
// 0 means total success.
//
static int ldpc_check(uint8_t codeword[])
{
    int errors = 0;

    for (int m = 0; m < FT8_LDPC_M; ++m)
    {
        uint8_t x = 0;
        for (int i = 0; i < kFT8_LDPC_num_rows[m]; ++i)
        {
            x ^= codeword[kFT8_LDPC_Nm[m][i] - 1];
        }
        if (x != 0)
        {
            ++errors;
        }
    }
    return errors;
}

void bp_decode(float codeword[], int max_iters, uint8_t plain[], int *ok)
{
    float tov[FT8_LDPC_N][3];
    float toc[FT8_LDPC_M][7];

    int min_errors = FT8_LDPC_M;

    // initialize message data
    for (int n = 0; n < FT8_LDPC_N; ++n)
    {
        tov[n][0] = tov[n][1] = tov[n][2] = 0;
    }

    for (int iter = 0; iter < max_iters; ++iter)
    {
        // Do a hard decision guess (tov=0 in iter 0)
        int plain_sum = 0;
        for (int n = 0; n < FT8_LDPC_N; ++n)
        {
            plain[n] = ((codeword[n] + tov[n][0] + tov[n][1] + tov[n][2]) > 0) ? 1 : 0;
            plain_sum += plain[n];
        }

        if (plain_sum == 0)
        {
            // message converged to all-zeros, which is prohibited
            break;
        }

        // Check to see if we have a codeword (check before we do any iter)
        int errors = ldpc_check(plain);

        if (errors < min_errors)
        {
            // we have a better guess - update the result
            min_errors = errors;

            if (errors == 0)
            {
                break; // Found a perfect answer
            }
        }

        // Send messages from bits to check nodes
        for (int m = 0; m < FT8_LDPC_M; ++m)
        {
            for (int n_idx = 0; n_idx < kFT8_LDPC_num_rows[m]; ++n_idx)
            {
                int n = kFT8_LDPC_Nm[m][n_idx] - 1;
                // for each (n, m)
                float Tnm = codeword[n];
                for (int m_idx = 0; m_idx < 3; ++m_idx)
                {
                    if ((kFT8_LDPC_Mn[n][m_idx] - 1) != m)
                    {
                        Tnm += tov[n][m_idx];
                    }
                }
                toc[m][n_idx] = fast_tanh(-Tnm / 2);
            }
        }

        // send messages from check nodes to variable nodes
        for (int n = 0; n < FT8_LDPC_N; ++n)
        {
            for (int m_idx = 0; m_idx < 3; ++m_idx)
            {
                int m = kFT8_LDPC_Mn[n][m_idx] - 1;
                // for each (n, m)
                float Tmn = 1.0f;
                for (int n_idx = 0; n_idx < kFT8_LDPC_num_rows[m]; ++n_idx)
                {
                    if ((kFT8_LDPC_Nm[m][n_idx] - 1) != n)
                    {
                        Tmn *= toc[m][n_idx];
                    }
                }
                tov[n][m_idx] = -2 * fast_atanh(Tmn);
            }
        }
    }

    *ok = min_errors;
}

// Ideas for approximating tanh/atanh:
// * https://varietyofsound.wordpress.com/2011/02/14/efficient-tanh-computation-using-lamberts-continued-fraction/
// * http://functions.wolfram.com/ElementaryFunctions/ArcTanh/10/0001/
// * https://mathr.co.uk/blog/2017-09-06_approximating_hyperbolic_tangent.html
// * https://math.stackexchange.com/a/446411

static float fast_tanh(float x)
{
    if (x < -4.97f)
    {
        return -1.0f;
    }
    if (x > 4.97f)
    {
        return 1.0f;
    }
    float x2 = x * x;
    float a = x * (945.0f + x2 * (105.0f + x2));
    float b = 945.0f + x2 * (420.0f + x2 * 15.0f);
    return a / b;
}

static float fast_atanh(float x)
{
    float x2 = x * x;
    float a = x * (945.0f + x2 * (-735.0f + x2 * 64.0f));
    float b = (945.0f + x2 * (-1050.0f + x2 * 225.0f));
    return a / b;
}
