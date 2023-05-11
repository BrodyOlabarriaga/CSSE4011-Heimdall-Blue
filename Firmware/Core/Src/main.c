/* C Library Includes */

/* TI Library Includes */
#include <ti/drivers/UART.h>
#include <ti/drivers/gpio.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/TaskP.h>
#include <ti/common/mmwave_error.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/dpm/dpm.h>
#include <ti/control/mmwavelink/mmwavelink.h>

/* Personal Includes */
#include "uart.h"
#include "tpacket.h"
#include "utils.h"

/* Private macros */

/* Global Variables */
TaskP_Handle gMmwaveCtrlTask;
TaskP_Handle gMmwaveDpmTask;
MmwaveLink_CallbackCfg_t gLinkCallbackCfg;
DPM_ProcChainCfg gDpmCfg;

UART_Handle uartHandle;

/* MMWave Control Task */
void MmwaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    /* Initialize the mmWave control module */
    errCode = MMWave_init();
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Register a callback function for asynchronous events */
    errCode = MMWave_registerCallback(MMWave_EventMsg_TLV, (void*) &gLinkCallbackCfg, NULL);
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Start the mmWave control module */
    errCode = MMWave_start();
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Wait for the mmWave module to stop */
    while (1)
    {
        TaskP_yield();
    }
}

/* MMWave DPM Task */
void MmwaveDpmTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    /* Initialize the DPM module */
    errCode = DPM_init();
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Start the DPM module */
    errCode = DPM_start((void*) &gDpmCfg);
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Wait for the DPM module to stop */
    while (1)
    {
        TaskP_yield();
    }
}

/* Main Function */
int main(void) {
    int32_t errCode;
    TaskP_Params taskParams;

    /* Initialize the device-specific SOC module */
    SOC_init();

    /* Initialize the mailbox module */
    errCode = Mailbox_init(MAILBOX_TYPE_MSS);
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Initialize the pinmux module */
    errCode = Pinmux_init();
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Initialize the GPIO module */
    errCode = GPIO_init();
    if (errCode != 0)
    {
        DebugP_assert(0);
    }

    /* Initialize the ESM module */
    errCode = ESM_init(0U);
    if (errCode != ESM_OK)
    {
        DebugP_assert(0);
    }

    /* Initialize the mmWave link callback configuration */
    memset((void*) &gLinkCallbackCfg, 0, sizeof(MmwaveLink_CallbackCfg_t));
    gLinkCallbackCfg.appCallback = &MmwaveLinkCallbackFxn;
    gLinkCallbackCfg.arg = NULL;

    tp_packet_t TPacket;
    tp_buffer_t TBuffer;

    // Init uart
    if (uart_init(&uartHandle) != TRUE) {
        printf("Failed to init UART");
        return;
    }

    // Enter main while loop
    while (TRUE) {

        // Read the mmW readings and send data over UART
        read_mmwave_angle();

        read_mmwave_position();

        read_mmwave_velocity();

    }

}

void read_mmwave_angle(UART_Handle uartHandle) {

    uint8_t uart_buf[MMWAVELINK_UART_MAX_PAYLOAD_LEN];
    int32_t num_bytes;

    // Wait for angle data to be available
    do {
        num_bytes = UART_read(uartHandle, uart_buf, MMWAVELINK_UART_MAX_PAYLOAD_LEN);
    } while (num_bytes <= 0 || mmwavelink_isMMWaveLinkFrame(uart_buf) == false);

    // Parse angle data
    MMWAVELINK_MultiObjBeamFormingOutputMsg *msg = (MMWAVELINK_MultiObjBeamFormingOutputMsg *) uart_buf;
    uint16_t num_targets = msg->header.numTLVs;
    MMWAVELINK_MultiObjBeamFormingOutputObj *targets = (MMWAVELINK_MultiObjBeamFormingOutputObj *) &msg->tlv[0];

    // Print angle data
    for (uint16_t i = 0; i < num_targets; i++) {
        float angle = targets[i].angleEst;
        float snr = targets[i].snr;
        System_printf("Target %u: Angle = %.2f degrees, SNR = %.2f dB\n", i+1, angle, snr);
    }
}

void read_mmwave_velocity(UART_Handle uartHandle) {

    uint8_t uart_buf[MMWAVELINK_UART_MAX_PAYLOAD_LEN];
    int32_t num_bytes;

    // Wait for velocity data to be available
    do {
        num_bytes = UART_read(uartHandle, uart_buf, MMWAVELINK_UART_MAX_PAYLOAD_LEN);
    } while (num_bytes <= 0 || mmwavelink_isMMWaveLinkFrame(uart_buf) == false);

    // Parse velocity data
    MMWAVELINK_MultiObjDetectionOutputMsg *msg = (MMWAVELINK_MultiObjDetectionOutputMsg *) uart_buf;
    uint16_t num_targets = msg->header.numObj;
    MMWAVELINK_MultiObjDetectionOutputObj *targets = (MMWAVELINK_MultiObjDetectionOutputObj *) &msg->obj[0];

    // Print velocity data
    for (uint16_t i = 0; i < num_targets; i++) {
        float vel = targets[i].dopplerIdx;
        float snr = targets[i].snr;
        System_printf("Target %u: Velocity = %.2f m/s, SNR = %.2f dB\n", i+1, vel, snr);
    }
}

void read_mmwave_position(UART_Handle uartHandle) {

    uint8_t uart_buf[MMWAVELINK_UART_MAX_PAYLOAD_LEN];
    int32_t num_bytes;

    // Wait for position data to be available
    do {
        num_bytes = UART_read(uartHandle, uart_buf, MMWAVELINK_UART_MAX_PAYLOAD_LEN);
    } while (num_bytes <= 0 || mmwavelink_isMMWaveLinkFrame(uart_buf) == false);

    // Parse position data
    MMWAVELINK_StatsOutputMsg *msg = (MMWAVELINK_StatsOutputMsg *) uart_buf;
    float pos_x = msg->stats[0].sum/100.0f;
    float pos_y = msg->stats[1].sum/100.0f;
    float pos_z = msg->stats[2].sum/100.0f;

    // Print position data
    System_printf("Position: x = %.2f m, y = %.2f m, z = %.2f m\n", pos_x, pos_y, pos_z);
}
