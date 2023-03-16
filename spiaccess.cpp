/**
 * @file spiaccess.hpp
 * @author melektron
 * @brief provides a mutext that should be locked before accessing SPI DMA (or calling any KIPR function)
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */

#include "spiaccess.hpp"

namespace kp
{
    std::mutex spi_access_mutex;

    std::unique_lock<std::mutex> aquireSPIAccess()
    {
        return std::unique_lock<std::mutex>(spi_access_mutex);
    }
};