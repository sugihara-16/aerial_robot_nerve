/*
 * flashmemory.cpp
 *
 *  Created on: 2016/11/18
 *      Author: anzai
 */
#include "flashmemory.h"
#include <string.h>

namespace FlashMemory {
  namespace {
    struct Data {
      void* ptr;
      size_t size;
      Data (void* ptr, size_t size):ptr(ptr), size(size){}
    };
    std::vector<Data> data;
    uint32_t m_data_address, m_data_sector;
    constexpr int FLASH_TIMEOUT_VALUE = 50000; //50s
    bool lock = false;
  }

  void init(uint32_t data_address, uint32_t data_sector){
    m_data_address = data_address;
    m_data_sector = data_sector;
  }

  void addValue(void* ptr, size_t size){
    Data tmp_data(ptr, size);
    data.push_back(tmp_data);
  }

  HAL_StatusTypeDef read(){
    HAL_StatusTypeDef status = HAL_ERROR;
#ifdef STM32F7
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
#endif
#ifdef STM32H7
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_2);
    // BANK1 cuases the flash failure by STLink from the second time. So we use BANK_2, which is tested OK
#endif

    uint32_t data_address = m_data_address;
    if (status == HAL_OK){
      for (unsigned int i = 0; i != data.size(); i++){
        memcpy(data[i].ptr, reinterpret_cast<void*>(data_address), data[i].size);
        data_address += data[i].size;
      }
    }

    /* If the program operation is completed, disable the PG Bit */
#ifdef STM32F7
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
#endif
#ifdef STM32H7
    CLEAR_BIT(FLASH->CR2, FLASH_CR_PG); // we assume to use bank2
#endif

    return status;
  }

  HAL_StatusTypeDef erase(){
    lock = true;

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
      lock = false;
      return status;
    }

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
#ifdef STM32H7
    EraseInitStruct.Banks = FLASH_BANK_2;
#endif
    EraseInitStruct.Sector = m_data_sector;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    const HAL_StatusTypeDef lock_status = HAL_FLASH_Lock();
    if (status == HAL_OK) status = lock_status;

    // Keep publishing paused only between a successful erase and its write.
    // A failed flash operation must not leave the ROS task disabled forever.
    if (status != HAL_OK) lock = false;
    return status;
  }

  HAL_StatusTypeDef write(){
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
      lock = false;
      return status;
    }

    uint32_t data_address = m_data_address;
#ifdef STM32F7
    for (unsigned int i = 0; i != data.size() && status == HAL_OK; i++){
      for (unsigned int j = 0; j != data[i].size && status == HAL_OK; j++) {
        status = HAL_FLASH_Program(
          FLASH_TYPEPROGRAM_BYTE,
          data_address + j,
          *(static_cast<uint8_t*>(data[i].ptr) + j));
      }
      data_address += data[i].size;
    }
#endif

#ifdef STM32H7

    /* the minimum write size in STM32H7 is FLASH_TYPEPROGRAM_FLASHWORD, which is size of 256 bit = 32byte. So we have to carefully calcuate the required amount of FLASHWORD to write. */
    uint8_t temp_data[FLASHWORD_SIZE];
    memset(temp_data, 0, FLASHWORD_SIZE);
    uint8_t byte_cnt = 0;

    for (unsigned int i = 0; i != data.size() && status == HAL_OK; i++){
      for (unsigned int j = 0; j != data[i].size && status == HAL_OK; j++) {
        memcpy(temp_data + byte_cnt, static_cast<uint8_t*>(data[i].ptr) + j, 1);
        byte_cnt++;
        if (byte_cnt == FLASHWORD_SIZE){
          status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            data_address,
            reinterpret_cast<uint32_t>(temp_data));
          data_address += 32;
          byte_cnt = 0;
        }
      }
    }
    if (byte_cnt > 0 && status == HAL_OK){
      status = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_FLASHWORD,
        data_address,
        reinterpret_cast<uint32_t>(temp_data)); // residual data
    }

#endif

    const HAL_StatusTypeDef lock_status = HAL_FLASH_Lock();
    if (status == HAL_OK) status = lock_status;

    lock = false;
    return status;
  }

  bool isLock() {
    return lock;
  }
}


