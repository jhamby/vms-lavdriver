/* Test the Load Average driver by reading from it.
 * You'll need to compile with /FLOAT=G_FLOAT to use the old driver,
 * and with /FLOAT=IEEE_FLOAT for the new, fixed-point driver.
 */

#define __NEW_STARLET 1
#include <descrip.h>
#include <iodef.h>
#include <stsdef.h>
#include <starlet.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]) {

#if __IEEE_FLOAT == 1
    $DESCRIPTOR (devnam, "LAX0:");
    uint32_t avgs[9] = { 0 };
#else
    $DESCRIPTOR (devnam, "LAV0:");
    float avgs[9] = { 0.0 };
#endif
    int status;
    unsigned short channel;

    /* assign the channel */
    status = sys$assign(&devnam, &channel, 0, NULL);
    if (!$VMS_STATUS_SUCCESS(status)) {
	fprintf(stderr, "test-lav-driver $assign err\n");
	return status;
    }

    /* read the bytes */
    status = sys$qiow(0, channel, IO$_READVBLK, NULL, NULL, 0,
                      avgs, sizeof(avgs), 0, 0, 0, 0);
    if (!$VMS_STATUS_SUCCESS(status)) {
	fprintf(stderr, "test-lav-driver $qiow err\n");
	return status;
    }

    /* deassign the channel */
    status = sys$dassgn(channel);

    /* print the output */
#if __IEEE_FLOAT == 1
    static const double scale = (1.0 / (1 << 14));
    printf("load average: %-12g %-12g %-12g\n",
	((double)avgs[0] * scale), ((double)avgs[1] * scale), ((double)avgs[2] * scale));
    printf("avg priority: %-12g %-12g %-12g\n",
	((double)avgs[3] * scale), ((double)avgs[4] * scale), ((double)avgs[5] * scale));
    printf("av dsk q len: %-12g %-12g %-12g\n",
	((double)avgs[6] * scale), ((double)avgs[7] * scale), ((double)avgs[8] * scale));
#else
    printf("load average: %-13g %-13g %-13g\n", avgs[0], avgs[1], avgs[2]);
    printf("avg priority: %-13g %-13g %-13g\n", avgs[3], avgs[4], avgs[5]);
    printf("av dsk q len: %-13g %-13g %-13g\n", avgs[6], avgs[7], avgs[8]);
#endif
}
