#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <float.h>
#define PI 3.14159265358979323846f
typedef struct { void *Instance; } CAN_HandleTypeDef;
typedef struct { uint8_t Data[8]; } Stub_Rx_Buffer;
typedef struct CAN_Manage_Object { CAN_HandleTypeDef *CAN_Handler; Stub_Rx_Buffer Rx_Buffer; } CAN_Manage_Object;
extern CAN_Manage_Object CAN1_Manage_Object, CAN2_Manage_Object;
#define CAN1 ((void*)1)
#define CAN2 ((void*)2)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
#include "alg_math.h"
#include "mi_motor.h"
