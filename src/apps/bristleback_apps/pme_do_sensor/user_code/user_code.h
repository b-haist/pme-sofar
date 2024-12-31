#include "serial.h"
#include "bsp.h"
#include "debug_configuration.h"

#pragma once

//holds the unix eopch time of the last wipe (P.F.)
// changed from int to float to match member function in configuration.h (B.H)
static float lastWipeTime = 0;
//Line terminator for userConfigurationPartition (B.H.)
static u_int32_t line_term_config = 13; // Carraige Return, CR, 0x0D
static u_int32_t bm_log_enable = true;



void setup(void);
void loop(void);
