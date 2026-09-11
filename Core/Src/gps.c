/*
 * mmc5983ma.c
 *
 *  Created on: 22 thg 8, 2026
 *      Author: khanh
 */

#include "gps.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Kích thước bộ đệm chứa một câu NMEA */
#define GPS_BUFFER_SIZE 128

/* UART được sử dụng cho GPS */
static UART_HandleTypeDef *gps_uart;

/* Bộ đệm nhận dữ liệu NMEA */
static char line_buffer[GPS_BUFFER_SIZE];

/* Vị trí hiện tại trong bộ đệm */
static uint16_t line_index = 0;

/* Biến lưu dữ liệu GPS */
static GPS_Data_t gps_data;

/**
 * @brief  Chuyển tọa độ NMEA sang độ thập phân
 * @param  str: chuỗi tọa độ dạng ddmm.mmmm hoặc dddmm.mmmm
 * @retval Tọa độ dạng degree
 *
 * Ví dụ:
 * 2030.1234 -> 20.502056 degree
 */
static double NMEA_To_Degree(const char *str)
{
    /* Chuyển chuỗi sang số */
    double raw = atof(str);

    /* Lấy phần độ */
    int degree = (int)(raw / 100);

    /* Lấy phần phút */
    double minute = raw - (degree * 100);

    /* Đổi phút sang độ */
    return degree + minute / 60.0;
}

/**
 * @brief  Khởi tạo thư viện GPS
 * @param  huart: UART dùng để giao tiếp với GPS
 * @retval HAL_OK
 */
HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *huart)
{
    /* Lưu UART để thư viện sử dụng */
    gps_uart = huart;

    /* Xóa dữ liệu GPS cũ */
    memset(&gps_data, 0, sizeof(gps_data));

    return HAL_OK;
}

/**
 * @brief  Phân tích câu NMEA GNRMC
 * @param  sentence: chuỗi GNRMC cần phân tích
 *
 * GNRMC chứa:
 * - Trạng thái GPS
 * - Latitude
 * - Longitude
 * - Speed
 * - Course
 */
static void GPS_ParseRMC(char *sentence)
{
    char *token;

    uint8_t field = 0;

    char lat_str[16] = {0};
    char lon_str[16] = {0};

    char lat_dir = 'N';
    char lon_dir = 'E';

    /* Lấy trường đầu tiên */
    token = strtok(sentence, ",");

    /* Duyệt từng trường trong câu NMEA */
    while (token != NULL)
    {
        switch (field)
        {
        case 2:

            /* A = dữ liệu hợp lệ, V = không hợp lệ */
            gps_data.fix = (token[0] == 'A');

            break;

        case 3:

            /* Lưu latitude dạng chuỗi */
            strcpy(lat_str, token);

            break;

        case 4:

            /* N = Bắc, S = Nam */
            lat_dir = token[0];

            break;

        case 5:

            /* Lưu longitude dạng chuỗi */
            strcpy(lon_str, token);

            break;

        case 6:

            /* E = Đông, W = Tây */
            lon_dir = token[0];

            break;

        case 7:

            /*
             * Speed trong RMC là knot
             *
             * 1 knot = 1.852 km/h
             */
            gps_data.speed_kmh =
                atof(token) * 1.852f;

            /* Đổi km/h -> m/s */
            gps_data.speed_ms =
                gps_data.speed_kmh / 3.6f;

            break;

        case 8:

            /*
             * Course Over Ground
             *
             * 0   = Bắc
             * 90  = Đông
             * 180 = Nam
             * 270 = Tây
             */
            gps_data.course = atof(token);

            break;
        }

        /* Chuyển sang trường tiếp theo */
        token = strtok(NULL, ",");

        field++;
    }

    /* Chuyển latitude sang độ thập phân */
    gps_data.latitude =
        NMEA_To_Degree(lat_str);

    /* Chuyển longitude sang độ thập phân */
    gps_data.longitude =
        NMEA_To_Degree(lon_str);

    /* Đảo dấu nếu ở Nam */
    if (lat_dir == 'S')
    {
        gps_data.latitude *= -1.0;
    }

    /* Đảo dấu nếu ở Tây */
    if (lon_dir == 'W')
    {
        gps_data.longitude *= -1.0;
    }
}

/**
 * @brief  Nhận từng byte từ UART GPS
 * @param  data: byte dữ liệu nhận được
 *
 * @note
 * Hàm này gom từng byte thành một câu NMEA.
 * Khi gặp '\n' thì coi như đã nhận xong một câu.
 */
void GPS_RxCallback(uint8_t data)
{
    /* Kết thúc một câu NMEA */
    if (data == '\n')
    {
        /* Kết thúc chuỗi */
        line_buffer[line_index] = 0;

        /* Phân tích câu vừa nhận */
        GPS_Process();

        /* Chuẩn bị nhận câu tiếp theo */
        line_index = 0;
    }
    else
    {
        /* Không cho phép tràn buffer */
        if (line_index < GPS_BUFFER_SIZE - 1)
        {
            /* Lưu byte vào buffer */
            line_buffer[line_index++] = data;
        }
    }
}

/**
 * @brief  Phân tích câu NMEA trong buffer
 *
 * @note
 * Hiện tại chỉ xử lý:
 * $GNRMC
 *
 * Nếu muốn đọc thêm GGA/GSA/GST...
 * có thể thêm điều kiện ở đây.
 */
void GPS_Process(void)
{
    /* Kiểm tra có phải câu GNRMC không */
    if (strncmp(line_buffer, "$GNRMC", 6) == 0)
    {
        char temp[GPS_BUFFER_SIZE];

        /* strtok() sẽ thay đổi chuỗi,
           nên tạo một bản sao */
        strcpy(temp, line_buffer);

        /* Phân tích dữ liệu RMC */
        GPS_ParseRMC(temp);
    }
}

/**
 * @brief  Lấy dữ liệu GPS hiện tại
 * @retval Con trỏ tới gps_data
 */
GPS_Data_t *GPS_GetData(void)
{
    return &gps_data;
}