# CPT212B touch controller driver
## Introduction
This package contains the CPT212B touch controller driver

The touch controller driver package includes cpt212b_driver.h, cpt212b_driver.c 
and cpt212b_defs.h files.

## Integration details
* Integrate cpt212b_driver.h, cpt212b_driver.c and cpt212b_defs.h in to the 
project.
* Include the cpt212b_driver.h file in your code like below.
``` c
#include "cpt212b_driver.h"
```

## File information
* cpt212b_driver.h : This header file contains the declarations of the touch controller driver APIs.
* cpt212b_driver.c : This source file contains the definitions of the sensor driver APIs.
* cpt212b_defs.h : This header file has the constants, macros and datatype declarations.


## Usage guide
### Initializing the touch controller
To initialize the touch controller, user need to create a device structure. User can do this by 
creating an instance of the structure cpt212b_dev_t. After creating the device strcuture, user 
need to fill in the various parameters as shown below.

#### Example of initialization touch controller device structure
``` c
static cpt212b_dev_t touch_controller = {
    .intrface = {
        .i2c_read = user_i2c_read,
        .i2c_write = user_i2c_write,
        .pin_output_set = user_pin_output_set,
        .delay_ms = user_delay_ms
    }
};
```
### Initializing the touch controller configuration profile
To initialize the touch controller configuration profile, user need to create a configuration profile structure. User can do this by 
creating an instance of the structure cpt212b_config_profile_t. After creating the configuration profile strcuture, user 
need to fill in the various parameters. User is able do it manually, or use the default configurations that are defined as CPT212B_DEFAULT_CONFIG_PROFILE as shown below
#### Example of initialization touch controller configuration profile structure
``` c
cpt212b_config_profile_t config = CPT212B_DEFAULT_CONFIG_PROFILE;
//! Edit default configuration profile
//! Change slave address
config.i2c_slave_address = 0xD0;
//! Turn off buzzer
config.buzzer_output_duration = 0;
config.buzzer_output_frequency = 0;
config.buzzer_drive_strength = 0;
```
### Touch controller packets reading
``` c
cpt212b_err_t rslt;
cpt212b_config_profile_t config = CPT212B_DEFAULT_CONFIG_PROFILE;
cpt212b_packet_t cpt212b_packet = {0};
static cpt212b_dev_t touch_controller = {
    .intrface = {
        .i2c_read = user_i2c_read,
        .i2c_write = user_i2c_write,
        .pin_output_set = user_pin_output_set,
        .delay_ms = user_delay_ms
    }
};

i2c_master_driver_initialize();

rslt = cpt212b_config_upload(&touch_controller, &config);
printf("Configuration upload result: %d\r\n", rslt);

if(rslt == CPT212B_OK) {
    printf("Configuration upload success, enter sensing mode\r\n");
    rslt = cpt212b_enter_sensing_mode(&touch_controller);
    if(rslt == CPT212B_OK) {
        printf("Sensing mode is active\r\n");
        gpio_mode(ITR_PIN_NUM, GPIO_MODE_INPUT);
    }
}
else {
    printf("Configuration upload failed, result %d\r\n", rslt);
}

//! Application main loop
while(1) {
    if(gpio_get_level(ITR_PIN_NUM) == 0) {
        rslt = cpt212b_get_packet(&touch_controller,
                            &cpt212b_packet);
        if(rslt == CPT212B_OK) {
            printf("----------------------------------------\r\n");
            printf("packet_counter: %d\r\n", cpt212b_packet.packet_counter);
            printf("event_type: %s\r\n",
                    cpt212b_packet.event_type == CPT212B_TOUCH_EVENT ? "TOUCH_EVENT" :
                    cpt212b_packet.event_type == CPT212B_RELEASE_EVENT ? "RELEASE_EVENT" :
                            "PROXIMITY_EVENT");
            printf("cs_index: %d\r\n", cpt212b_packet.cs_index);
        }
        else {
            printf("Getting packet failed, result %d\r\n", rslt);
        }
    }
}
```
#### Output
```
Configuration upload result: 0
Configuration upload success, enter sensing mode
Sensing mode is active
----------------------------------------
packet_counter: 0
event_type: TOUCH_EVENT
cs_index: 1
----------------------------------------
packet_counter: 1
event_type: RELEASE_EVENT
cs_index: 1
```
## Contributing
Pull requests are welcome. For major changes, please open an issue first to
discuss what you would like to change.

Please make sure to update tests as appropriate.
