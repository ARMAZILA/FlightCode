/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L3GD20__H
#define __L3GD20__H

bool l3gd20Detect(void);

// Read 3 gyro values into user-provided buffer.
void l3gd20Read(int16_t *gyroData);

// Return FIFO stored data level
uint8_t l3gd20GetFIFOLevel(void);

// Return gyro temperature
uint8_t l3gd20GetTemp(int16_t * temp);

#endif
