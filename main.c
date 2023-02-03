#include <stdio.h>
#include <string.h>
#include "tusb.h"
#include "pdnd/pdnd.h"
#include "pdnd/pdnd_display.h"
#include "hardware/pio.h"
#include "pdnd/pio/pio_spi.h"

//#include "swd.h"

const uint32_t OUT_TARGET_POWER = 0;
const uint32_t IN_STM_RST = 1;
const uint32_t TARGET_POWER = 18;

// SWD Code -- Move into separate C file later...

#define SWDIO_Pin 14
#define SWCLK_Pin 15



/* Internal SWD status. There exist combined SWD status values (e.g. 0x60), since subsequent command replys are OR'ed. Thus there exist cases where the previous command executed correctly (returned 0x20) and the following command failed (returned 0x40), resulting in 0x60. */
typedef enum {
    // TODO: 0xA0 fehlt.
    swdStatusNone = 0x00u,    /* No status available (yet) */
    swdStatusOk = 0x20u,      /* Status OK */
    swdStatusWait = 0x40u,    /* Wait/Retry requested (bus access was not granted in time) */
    swdStatusWaitOK = 0x60u,  /* Wait requested + additional OK (previous command OK, but no bus access) */
    swdStatusFault = 0x80u,   /* Fault during command execution (command error (access denied etc.)) */
    swdStatusFaultOK = 0xA0u, /* Fault during command execution, previous command was successful */
    swdStatusFailure = 0xE0u  /* Failure during communication (check connection, no valid status reply received) */
} swdStatus_t;

typedef enum {
    swdPortSelectDP = 0x00u,
    swdPortSelectAP = 0x01u
} swdPortSelect_t;

typedef enum {
    swdAccessDirectionWrite = 0x00u,
    swdAccessDirectionRead = 0x01u
} swdAccessDirection_t;


#define MWAIT __asm__ __volatile__( \
    ".syntax unified 		\n"          \
    "	movs r0, #0x20 		\n"          \
    "1: 	subs r0, #1 		\n"          \
    "	bne 1b 			\n"                 \
    ".syntax divided"               \
    :                               \
    :                               \
    : "cc", "r0")

#define N_READ_TURN (9u)

static uint8_t swdParity(uint8_t const* data, uint8_t const len);
static void swdDatasend(uint8_t const* data, uint8_t const len);
static void swdDataIdle(void);
static void swdDataPP(void);
static void swdTurnaround(void);
static void swdReset(void);
static void swdDataRead(uint8_t* const data, uint8_t const len);
static void swdBuildHeader(swdAccessDirection_t const adir, swdPortSelect_t const portSel, uint8_t const A32, uint8_t* const header);
static swdStatus_t swdReadPacket(swdPortSelect_t const portSel, uint8_t const A32, uint32_t* const data);
static swdStatus_t swdWritePacket(swdPortSelect_t const portSel, uint8_t const A32, uint32_t const data);
static swdStatus_t swdReadAP0(uint32_t* const data);

static uint8_t swdParity(uint8_t const* data, uint8_t const len) {
    uint8_t par = 0u;
    uint8_t cdata = 0u;
    uint8_t i;

    for (i = 0u; i < len; ++i) {
        if ((i & 0x07u) == 0u) {
            cdata = *data;
            ++data;
        }

        par ^= (cdata & 0x01u);
        cdata >>= 1u;
    }

    return par;
}

static void swdDatasend(uint8_t const* data, uint8_t const len) {
    uint8_t cdata = 0u;
    uint8_t i;

    for (i = 0u; i < len; ++i) {
        if ((i & 0x07u) == 0x00u) {
            cdata = *data;
            ++data;
        }

        if ((cdata & 0x01u) == 0x01u) {
            gpio_put(SWDIO_Pin, 1);
        } else {
            gpio_put(SWDIO_Pin, 0);
        }
        MWAIT;

        gpio_put(SWCLK_Pin, 1);
        MWAIT;
        gpio_put(SWCLK_Pin, 0);
        cdata >>= 1u;
        MWAIT;
    }
}

static void swdDataIdle(void) {
    gpio_put(SWDIO_Pin, 1);
    MWAIT;
    gpio_set_dir(SWDIO_Pin, GPIO_IN);
    MWAIT;
}

static void swdDataPP(void) {
    MWAIT;
    gpio_put(SWDIO_Pin, 0);
    gpio_set_dir(SWDIO_Pin, GPIO_OUT);
    MWAIT;
}

static void swdTurnaround(void) {
    gpio_put(SWCLK_Pin, 1);
    MWAIT;
    gpio_put(SWCLK_Pin, 0);
    MWAIT;
}

static void swdDataRead(uint8_t* const data, uint8_t const len) {
    uint8_t i;
    uint8_t cdata = 0u;

    MWAIT;
    swdDataIdle();
    MWAIT;

    for (i = 0u; i < len; ++i) {
        cdata >>= 1u;
        cdata |= gpio_get(SWDIO_Pin) ? 0x80u : 0x00u;
        data[(((len + 7u) >> 3u) - (i >> 3u)) - 1u] = cdata;

        gpio_put(SWCLK_Pin, 1);
        MWAIT;
        gpio_put(SWCLK_Pin, 0);
        MWAIT;

        /* clear buffer after reading 8 bytes */
        if ((i & 0x07u) == 0x07u) {
            cdata = 0u;
        }
    }
}

static void swdReset(void) {
    uint8_t i;

    MWAIT;
    gpio_put(SWCLK_Pin, 1);
    gpio_put(SWDIO_Pin, 1);
    MWAIT;

    /* 50 clk+x */
    for (i = 0u; i < (50u + 10u); ++i) {
        gpio_put(SWCLK_Pin, 1);
        MWAIT;
        gpio_put(SWCLK_Pin, 0);
        MWAIT;
    }

    gpio_put(SWDIO_Pin, 0);

    for (i = 0u; i < 3u; ++i) {
        gpio_put(SWCLK_Pin, 1);
        MWAIT;
        gpio_put(SWCLK_Pin, 0);
        MWAIT;
    }
}

static void swdBuildHeader(swdAccessDirection_t const adir, swdPortSelect_t const portSel, uint8_t const A32, uint8_t* const header) {
    if (portSel == swdPortSelectAP) {
        *header |= 0x02u; /* Access AP */
    }

    if (adir == swdAccessDirectionRead) {
        *header |= 0x04u; /* read access */
    }

    switch (A32) {
        case 0x01u:
            *header |= 0x08u;
            break;

        case 0x02u:
            *header |= 0x10u;
            break;

        case 0x03u:
            *header |= 0x18u;
            break;

        default:
        case 0x00u:

            break;
    }

    *header |= swdParity(header, 7u) << 5u;
    *header |= 0x01u; /* startbit */
    *header |= 0x80u;
}

static swdStatus_t swdReadPacket(swdPortSelect_t const portSel, uint8_t const A32, uint32_t* const data) {
    swdStatus_t ret;
    uint8_t header = 0x00u;
    uint8_t rp[1] = {0x00u};
    uint8_t resp[5] = {0u};
    uint8_t i;

    swdBuildHeader(swdAccessDirectionRead, portSel, A32, &header);

    swdDatasend(&header, 8u);
    swdDataIdle();
    swdTurnaround();
    swdTurnaround();
    swdTurnaround();
    swdTurnaround();
    swdDataRead(rp, 3u);

    swdDataRead(resp, 33u);

    swdDataPP();

    for (i = 0u; i < N_READ_TURN; ++i) {
        swdTurnaround();
    }

    *data = resp[4] | (resp[3] << 8u) | (resp[2] << 16u) | (resp[1] << 24u);

    ret = rp[0];

    return ret;
}

static swdStatus_t swdWritePacket(swdPortSelect_t const portSel, uint8_t const A32, uint32_t const data) {
    swdStatus_t ret;
    uint8_t header = 0x00u;
    uint8_t rp[1] = {0x00u};
    uint8_t data1[5] = {0u};
    uint8_t i;

    swdBuildHeader(swdAccessDirectionWrite, portSel, A32, &header);

    swdDatasend(&header, 8u);
    MWAIT;

    swdDataIdle();
    MWAIT;

    swdTurnaround();

    swdDataRead(rp, 3u);

    swdDataIdle();

    swdTurnaround();
    swdDataPP();

    data1[0] = data & 0xFFu;
    data1[1] = (data >> 8u) & 0xFFu;
    data1[2] = (data >> 16u) & 0xFFu;
    data1[3] = (data >> 24u) & 0xFFu;
    data1[4] = swdParity(data1, 8u * 4u);

    swdDatasend(data1, 33u);

    swdDataPP();

    for (i = 0u; i < 20u; ++i) {
        swdTurnaround();
    }

    ret = rp[0];

    return ret;
}

swdStatus_t swdReadIdcode(uint32_t* const idCode) {
    uint32_t ret;

    ret = swdReadPacket(swdPortSelectDP, 0x00u, idCode);

    return ret;
}

swdStatus_t swdSelectAPnBank(uint8_t const ap, uint8_t const bank) {
    swdStatus_t ret = swdStatusNone;
    uint32_t data = 0x00000000u;

    data |= (uint32_t)(ap & 0xFFu) << 24u;
    data |= (uint32_t)(bank & 0x0Fu) << 0u;

    /* write to select register */
    ret |= swdWritePacket(swdPortSelectDP, 0x02u, data);

    return ret;
}

static swdStatus_t swdReadAP0(uint32_t* const data) {
    swdStatus_t ret = swdStatusNone;

    swdReadPacket(swdPortSelectAP, 0x00u, data);

    return ret;
}

swdStatus_t swdSetAP32BitMode(uint32_t* const data) {
    swdStatus_t ret = swdStatusNone;

    swdSelectAPnBank(0x00u, 0x00u);

    uint32_t d = 0u;

    ret |= swdReadAP0(&d);

    ret |= swdReadPacket(swdPortSelectDP, 0x03u, &d);

    d &= ~(0x07u);
    d |= 0x02u;

    ret |= swdWritePacket(swdPortSelectAP, 0x00u, d);

    ret |= swdReadAP0(&d);
    ret |= swdReadPacket(swdPortSelectDP, 0x03u, &d);

    if (data != NULL) {
        *data = d;
    }

    return ret;
}

swdStatus_t swdSelectAHBAP(void) {
    swdStatus_t ret = swdSelectAPnBank(0x00u, 0x00u);

    return ret;
}

swdStatus_t swdReadAHBAddr(uint32_t const addr, uint32_t* const data) {
    swdStatus_t ret = swdStatusNone;
    uint32_t d = 0u;

    ret |= swdWritePacket(swdPortSelectAP, 0x01u, addr);

    ret |= swdReadPacket(swdPortSelectAP, 0x03u, &d);
    ret |= swdReadPacket(swdPortSelectDP, 0x03u, &d);

    *data = d;

    return ret;
}

swdStatus_t swdEnableDebugIF(void) {
    swdStatus_t ret = swdStatusNone;

    ret |= swdWritePacket(swdPortSelectDP, 0x01u, 0x50000000u);

    return ret;
}

swdStatus_t swdInit(uint32_t* const idcode) {
    swdStatus_t ret = swdStatusNone;

    swdReset();
    ret |= swdReadIdcode(idcode);

    return ret;
}

void initialize_board() {
    // Configure GPIO power, connected to N channel mosfet through logic level shifter
    gpio_init(TARGET_POWER);
    gpio_put(TARGET_POWER, 0);
    gpio_set_dir(TARGET_POWER, GPIO_OUT);
    gpio_pull_up(TARGET_POWER);
    // Glitch line, connected to VCAP through N channel MOSFET to GND when input is high
    gpio_init(PDND_GLITCH);
    gpio_put(PDND_GLITCH, 0);
    gpio_set_dir(PDND_GLITCH, GPIO_OUT);
    // Trigger line, here we are triggering on the reset line
    gpio_init(IN_STM_RST);
    gpio_set_dir(IN_STM_RST, GPIO_IN);
    gpio_disable_pulls(IN_STM_RST);
    // Configure SWD Pins
    gpio_init(SWDIO_Pin);
    gpio_set_dir(SWDIO_Pin,GPIO_OUT);
    gpio_init(SWCLK_Pin);
    gpio_set_dir(SWCLK_Pin,GPIO_OUT);


}

//restart the Airtag
static inline power_cycle_target() {
    gpio_put(TARGET_POWER, 0);
    sleep_ms(250);
    gpio_put(TARGET_POWER, 1);
}

//Serial Connection Byte Commands to execute the corresponding actions in main()
const uint8_t CMD_DELAY = 0x41;
const uint8_t CMD_PULSE = 0x42;
const uint8_t CMD_GLITCH = 0x43;
const uint8_t CMD_POWERUP = 0x44;
const uint8_t CMD_POWERDOWN = 0x45;
const uint8_t CMD_SWD_ID = 0x49;

//Print function to print current stats to serial output
void dv(uint32_t delay, uint32_t pulse, uint32_t counter) {
    cls(false);
    printf("Try Number %d: Delay = %d , Pulse = %d \r", counter, delay, pulse);
}


//Main Function
int main() {
    stdio_init_all();
    // Sleep X MS to Wait for USB Serial to initialize <-- This means you will need to connect via serial before pico fully starts up (This is important to not encounter no serial found bug)
    while (!tud_cdc_connected()) { sleep_ms(100);  }
    printf("USB-Serial connected!\r\n");
    stdio_set_translate_crlf(&stdio_usb, false);
    pdnd_initialize();
    pdnd_enable_buffers(0);
    pdnd_display_initialize();
    pdnd_enable_buffers(1);
    cls(false);
    printf("Initializing Raspberry Pi Pico Board ... \r\n");

    // Sets up trigger & glitch output
    initialize_board();
    
    printf("Boot-Sequence finished - ready for Serial Commands \r\n\n");
    
    //Delay to wait after receiving the signal that airtag NRF has power (Delay = 10000 is 10000 ASM NOP executions on the Pico)
    uint32_t delay = 3400;
    
    //Length of the Glitch (how long power cut from cpu core) measured in ASM Nop executions
    uint32_t pulse = 18;
    uint32_t glitch_count = 0;
    
    //counter to count attempts
    uint32_t counter = 0;
    uint32_t idCode = 0;
    
    while(1) {
        counter = counter + 1;
        if((counter % 100) == 0){
            dv(delay, pulse, counter);
        }
        
        //Get Command from Serial and execute accordingly
        uint8_t cmd = getchar();
        switch(cmd) {
            //Set Delay
            case CMD_DELAY:
                fread(&delay, 1, 4, stdin);
                //dv(delay, pulse, counter);
                break;
            //Set Pulse
            case CMD_PULSE:
                fread(&pulse, 1, 4, stdin);
                //dv(delay, pulse, counter);
                break;
            //Command to Power up the Airtag 
            case CMD_POWERUP:
                gpio_put(TARGET_POWER, 1);
                break;
            //Command to Power down the Airtag 
            case CMD_POWERDOWN:
                gpio_put(TARGET_POWER, 0);
                break;
            //Command to execute glitch
            case CMD_GLITCH:
		glitch_count += 1;
                power_cycle_target();

		//Wait for reset to go high
                while(!gpio_get(IN_STM_RST));
		for(uint32_t i=0; i < delay; i++) {
                    asm("NOP");
                }
                
                //Trigger N-Channel-Mosfet for Pulse Number of ASM NOPs (cut power of cpu core for Pulse number of asm NOPs)
                gpio_put(PDND_GLITCH, 1);
                for(uint32_t i=0; i < pulse; i++) {
                    asm("NOP");
                }
                gpio_put(PDND_GLITCH, 0);

        }
    }

    return 0;
}

