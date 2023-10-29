/**
 * @file   chunk.hpp
 * @brief  Chunk Data Definitions.
 * @author Thomas Reidemeister <thomas@labforge.ca>
 *
 * Copyright (C) 2013-2022 Labforge Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __LF_GEV_CHUNK_HPP__
#define __LF_GEV_CHUNK_HPP__

#include <cstdint>

#define MAX_KEYPOINTS 0xFFFF
#define CHUNK_PADDING 64 // Extra size for GEV chunk headers

typedef enum {
  CHUNK_ID_FEATURES = 0x4001,
  CHUNK_ID_DESCRIPTORS = 0x4002,
} chunk_type_t;

typedef struct point_u16 {
  uint16_t x;  ///< x coordinate  of the point
  uint16_t y;  ///< y coordinate of the point
} point_u16_t; ///< a 2D uint16 point representation

typedef struct __attribute__((packed, aligned(4))) {
  uint8_t data[64];         ///< up to 486 bits of descriptor data LSB first
} descriptor_t;

typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  point_u16_t points[MAX_KEYPOINTS];
} keypoints_t;

typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  descriptor_t descriptors[MAX_KEYPOINTS];
} descriptors_t;

#endif // __LF_GEV_CHUNK_HPP__