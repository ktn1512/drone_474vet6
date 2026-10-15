/**
 * @file gps.c
 * @brief Gom byte GPS và chuyển các token GNRMC thành dữ liệu dùng chung.
 */

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
/**
 * @brief Dung lượng bộ đệm một câu GPS tính bằng byte, gồm byte kết thúc NUL.
 */
#define GPS_BUFFER_SIZE 128

/* UART được sử dụng cho GPS */
/**
 * @brief Handle UART được GPS_Init lưu; hiện chưa được dùng để tự nhận dữ liệu.
 */
static UART_HandleTypeDef *gps_uart;

/* Bộ đệm nhận dữ liệu NMEA */
/**
 * @brief Bộ đệm nhận câu GPS dùng chung; chỉ đặt kết thúc NUL khi nhận LF.
 */
static char line_buffer[GPS_BUFFER_SIZE];

/* Vị trí hiện tại trong bộ đệm */
/**
 * @brief Chỉ số byte kế tiếp trong line_buffer; giới hạn lưu đến GPS_BUFFER_SIZE - 1.
 */
static uint16_t line_index = 0;

/* Biến lưu dữ liệu GPS */
/**
 * @brief Dữ liệu GPS dùng chung được Init xóa và parser RMC cập nhật từng trường.
 */
static GPS_Data_t gps_data;

/**
 * @brief  Chuyển tọa độ NMEA sang độ thập phân
 * @param  str: chuỗi tọa độ dạng ddmm.mmmm hoặc dddmm.mmmm
 * @retval Tọa độ dạng degree
 *
 * Ví dụ:
 * 2030.1234 -> 20.502056 degree
 */
/**
 * @brief Đổi tọa độ dạng độ/phút NMEA sang độ thập phân chưa áp dụng hướng.
 * @param[in] str Chuỗi kết thúc NUL dạng ddmm.mmmm hoặc dddmm.mmmm.
 * @return Phần độ cộng phần phút chia 60, đơn vị độ thập phân.
 * @note Hàm dùng atof và không kiểm tra cú pháp hay phạm vi tọa độ.
 */
static double NMEA_To_Degree(const char *str)
{
    /* Chuyển chuỗi sang số */
    /**
     * @brief Tọa độ số ở dạng ddmm.mmmm/dddmm.mmmm sau atof.
     */
    double raw = atof(str);

    /* Lấy phần độ */
    /**
     * @brief Phần nguyên số độ được lấy trước phần phút.
     */
    int degree = (int)(raw / 100);

    /* Lấy phần phút */
    /**
     * @brief Phần phút còn lại của tọa độ NMEA, đơn vị phút cung.
     */
    double minute = raw - (degree * 100);

    /* Đổi phút sang độ */
    return degree + minute / 60.0;
}

/**
 * @brief  Khởi tạo thư viện GPS
 * @param  huart: UART dùng để giao tiếp với GPS
 * @retval HAL_OK
 */
/**
 * @brief Lưu UART và xóa cấu trúc dữ liệu GPS, không khởi động nhận.
 * @see GPS_Init trong gps.h để biết hợp đồng sử dụng.
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
/**
 * @brief Tách token RMC và cập nhật trạng thái, tọa độ, tốc độ, hướng chuyển động.
 * @param[in,out] sentence Chuỗi câu NUL-terminated có thể sửa; strtok ghi NUL vào các dấu phẩy.
 * @note strtok bỏ qua trường rỗng; chỉ số field ở đây là chỉ số token thực tế.
 * @note Hai token tọa độ phải vừa các bộ đệm 16 byte, gồm NUL; hàm dùng strcpy.
 * @note Không kiểm checksum; vẫn cập nhật các trường khi trạng thái không có fix.
 */
static void GPS_ParseRMC(char *sentence)
{
    /**
     * @brief Con trỏ token hiện tại trong câu bị strtok chia tại dấu phẩy.
     */
    char *token;

    /**
     * @brief Chỉ số token thực tế, bắt đầu 0 từ tên câu; trường rỗng bị strtok bỏ qua.
     */
    uint8_t field = 0;

    /**
     * @brief Chuỗi vĩ độ ddmm.mmmm; dung lượng 16 byte gồm NUL.
     */
    char lat_str[16] = {0};
    /**
     * @brief Chuỗi kinh độ dddmm.mmmm; dung lượng 16 byte gồm NUL.
     */
    char lon_str[16] = {0};

    /**
     * @brief Hướng vĩ độ; mặc định N và đảo dấu khi token hướng là S.
     */
    char lat_dir = 'N';
    /**
     * @brief Hướng kinh độ; mặc định E và đảo dấu khi token hướng là W.
     */
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
/**
 * @brief Gom byte, xử lý khi gặp LF và chuẩn bị nhận câu tiếp theo.
 * @see GPS_RxCallback trong gps.h để biết hợp đồng sử dụng.
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
/**
 * @brief Chỉ phân tích câu bắt đầu bằng $GNRMC từ bộ đệm hiện tại.
 * @see GPS_Process trong gps.h để biết hợp đồng sử dụng.
 */
void GPS_Process(void)
{
    /* Kiểm tra có phải câu GNRMC không */
    if (strncmp(line_buffer, "$GNRMC", 6) == 0)
    {
        /**
         * @brief Bản sao câu GPS để strtok có thể sửa mà giữ nguyên line_buffer.
         */
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
/**
 * @brief Trả địa chỉ cấu trúc dữ liệu GPS tĩnh.
 * @see GPS_GetData trong gps.h để biết hợp đồng sử dụng.
 */
GPS_Data_t *GPS_GetData(void)
{
    return &gps_data;
}