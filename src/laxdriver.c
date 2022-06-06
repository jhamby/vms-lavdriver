#pragma module LAXDRIVER "X-1"
/*
 * System load average extended driver (LAX0:), returning IEEE floats.
 * Copyright 2022, Jake Hamby.
 * MIT License.
 *
 * Some code comments are from the example device driver in sys$examples:
 * that this driver used as a template.
 */

/* Define system data structure types and constants */

#define __NEW_STARLET 1

#include <bufiodef.h>		/* Define the packet header for a system */
				/*   buffer for buffered I/O data */
#include <ccbdef.h>             /* Channel control block */
#include <crbdef.h>             /* Controller request block */
#include <dcdef.h>              /* Device codes */
#include <ddbdef.h>             /* Device data block */
#include <ddtdef.h>             /* Driver dispatch table */
#include <devdef.h>             /* Device characteristics */
#include <dptdef.h>             /* Driver prologue table */
#include <fdtdef.h>             /* Function decision table */
#include <fkbdef.h>             /* Fork block */
#include <idbdef.h>             /* Interrupt data block */
#include <iocdef.h>             /* IOC constants */
#include <iodef.h>              /* I/O function codes */
#include <irpdef.h>             /* I/O request packet */
#include <orbdef.h>             /* Object rights block */
#include <pcbdef.h>             /* Process control block */
#include <ssdef.h>              /* System service status codes */
#include <stsdef.h>             /* Status value fields */
#include <ucbdef.h>             /* Unit control block */
#include <aaa_system_cells.h>	/* Global data references */

/* Define function prototypes for system routines */

#include <exe_routines.h>       /* Prototypes for exe$ and exe_std$ routines */
#include <ioc_routines.h>       /* Prototypes for ioc$ and ioc_std$ routines */
#include <sch_routines.h>       /* Prototypes for sch$ and sch_std$ routines */

/* Define various device driver macros */

#include <vms_drivers.h>        /* Device driver support macros, including */
                                /* table initialization macros and prototypes */
#include <vms_macros.h>         /* Additional macros */

/* Define the DEC C functions used by this driver */

#include <string.h>             /* String routines provided by "kernel CRTL" */
#include <inttypes.h>		/* C99 typedefs (old header for compat.) */

/* Define Device-Dependent Unit Control Block with extensions for LAX device */

typedef struct {
    UCB		ucb$r_ucb;                   /* Generic UCB */
    float	ucb$f_avgs[9];
    uint32_t	ucb$l_priority_mask;
} LAX_UCB;


/* Driver table initialization routine */

    int  driver$init_tables (void);

/* Device I/O database structure initialization routine */

    void lax_struc_init (CRB *crb, DDB *ddb, IDB *idb, ORB *orb, LAX_UCB *ucb);

/* Device I/O database structure re-initialization routine */

    void lax_struc_reinit (CRB *crb, DDB *ddb, IDB *idb, ORB *orb, LAX_UCB *ucb);

/* Unit initialization routine */

    int  lax_unit_init (IDB *idb, LAX_UCB *ucb);

/* FDT routine for read functions */

    int  lax_read (IRP *irp, PCB *pcb, LAX_UCB *ucb, CCB *ccb);

/* FDT routine for write functions */

//    int  lax_write (IRP *irp, PCB *pcb, LAX_UCB *ucb, CCB *ccb);

/* Periodic load averages update via Fork-wait mechanism */

    void lax_update_stats_fork (void *fr3, void *fr4, LAX_UCB *ucb);

/* Define some constants for the load averaging multiplication factors. */

static const float old_lax_1min = 0.983471453;
static const float new_lax_1min = 0.016528547;

static const float old_lax_5min = 0.996672213;
static const float new_lax_5min = 0.003327787;

static const float old_lax_15min = 0.998889506;
static const float new_lax_15min = 0.001110494;


/*
 * DRIVER$INIT_TABLES - Initialize Driver Tables
 *
 * Functional description:
 *
 *   This routine completes the initialization of the DPT, DDT, and FDT
 *   structures.  If a driver image contains a routine named DRIVER$INIT_TABLES
 *   then this routine is called once by the $LOAD_DRIVER service immediately
 *   after the driver image is loaded or reloaded and before any validity checks
 *   are performed on the DPT, DDT, and FDT.  A prototype version of these
 *   structures is built into this image at link time from the
 *   VMS$VOLATILE_PRIVATE_INTERFACES.OLB library.  Note that the device related
 *   data structures (e.g. DDB, UCB, etc.) have not yet been created when this
 *   routine is called.  Thus the actions of this routine must be confined to
 *   the initialization of the DPT, DDT, and FDT structures which are contained
 *   in the driver image.
 *
 *   TODO: switch to VAX MACRO-based table initialization to reduce code size.
 *
 * Calling convention:
 *
 *   status = driver$init_tables ();
 *
 * Input parameters:
 *
 *   None.
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   status     If the status is not successful, then the driver image will
 *              be unloaded.  Note that the ini_* macros used below will
 *              result in a return from this routine with an error status if
 *              an initialization error is detected.
 *
 * Implicit inputs:
 *
 *  driver$dpt, driver$ddt, driver$fdt
 *              These are the externally defined names for the prototype
 *              DPT, DDT, and FDT structures that are linked into this driver.
 *
 * Environment:
 * 
 *   Kernel mode, system context.
 */
 

int driver$init_tables (void)  {

    /* Prototype driver DPT, DDT, and FDT will be pulled in from the
     * VMS$VOLATILE_PRIVATE_INTERFACES.OLB library at link time.
     */
    extern DPT driver$dpt;
    extern DDT driver$ddt;
    extern FDT driver$fdt;

    /* Finish initialization of the Driver Prologue Table (DPT) */

    ini_dpt_name        (&driver$dpt, "LAXDRIVER");
    ini_dpt_adapt       (&driver$dpt, AT$_NULL); /* software adapter */
    ini_dpt_defunits    (&driver$dpt, 1);
    ini_dpt_maxunits    (&driver$dpt, 1);
    ini_dpt_ucbsize     (&driver$dpt, sizeof(LAX_UCB));
    ini_dpt_struc_init  (&driver$dpt, lax_struc_init );
    ini_dpt_struc_reinit(&driver$dpt, lax_struc_reinit );
    ini_dpt_end         (&driver$dpt);

    /* Finish initialization of the Driver Dispatch Table (DDT) */

    ini_ddt_unitinit    (&driver$ddt, lax_unit_init);
    ini_ddt_cancel      (&driver$ddt, ioc_std$cancelio);
    ini_ddt_end         (&driver$ddt);

    /* Finish initialization of the Function Decision Table (FDT)   */
    /*								    */
    /* The BUFFERED_64 indicates that this driver supports a 64-bit */
    /* virtual address in the QIO P1 parameter for that function.   */
    /* This driver, therefore, supports 64-bit user buffers in all  */
    /* of its I/O functions.					    */

    ini_fdt_act (&driver$fdt, IO$_READLBLK, lax_read, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_READPBLK, lax_read, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_READVBLK, lax_read, BUFFERED_64);

// Note: writing the priority mask isn't supported.
//    ini_fdt_act (&driver$fdt, IO$_WRITELBLK, lax_write, BUFFERED_64);
//    ini_fdt_act (&driver$fdt, IO$_WRITEPBLK, lax_write, BUFFERED_64);
//    ini_fdt_act (&driver$fdt, IO$_WRITEVBLK, lax_write, BUFFERED_64);

    ini_fdt_end (&driver$fdt);

    /* If we got this far then everything worked, so return success. */

    return SS$_NORMAL;
}


/*
 * LR$STRUC_INIT - Device Data Structure Initialization Routine
 *
 * Functional description:
 *
 *   This routine is called once for each unit by the $LOAD_DRIVER service
 *   after that UCB is created.  At the point of this call the UCB has not
 *   yet been fully linked into the I/O database.  This routine is responsible
 *   for filling in driver specific fields that in the I/O database structures
 *   that are passed as parameters to this routine.
 *
 *   This routine is responsible for filling in the fields that are not
 *   affected by a RELOAD of the driver image.  In contrast, the structure
 *   reinitialization routine is responsible for filling in the fields that
 *   need to be corrected when (and if) this driver image is reloaded.
 *
 *   After this routine is called for a new unit, then the reinitialization
 *   routine is called as well.  Then the $LOAD_DRIVER service completes the
 *   integration of these device specific structures into the I/O database.
 *
 *   Note that this routine must confine its actions to filling in these I/O
 *   database structures and may not attempt to initialize the hardware device.
 *   Initialization of the hardware device is the responsibility of the 
 *   controller and unit initialization routines which are called some time
 *   later.
 *
 * Calling convention:
 *
 *   lax_struc_init (crb, ddb, idb, orb, ucb)
 *
 * Input parameters:
 *
 *   crb        Pointer to associated controller request block.
 *   ddb        Pointer to associated device data block.
 *   idb        Pointer to associated interrupt dispatch block.
 *   orb        Pointer to associated object rights block.
 *   ucb        Pointer to the unit control block that is to be initialized.
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   None.
 *
 * Environment:
 * 
 *   Kernel mode, system context, IPL may be as high as 31 and may not be
 *   altered.
 *
 */ 

void lax_struc_init (CRB *crb, DDB *ddb, IDB *idb, ORB *orb, LAX_UCB *ucb) {

    /* Initialize the fork lock and device IPL fields */

    ucb->ucb$r_ucb.ucb$b_flck = SPL$C_IOLOCK8;
    ucb->ucb$r_ucb.ucb$b_dipl = IOC$K_DEVICE_IPL;

    /* Device characteristics: Record oriented (REC), Available (AVL), 
     *	Input device (IDV), Shared (SHR)
     */
    ucb->ucb$r_ucb.ucb$l_devchar = DEV$M_REC | DEV$M_AVL | DEV$M_IDV | DEV$M_SHR;

    /* Set to prefix device name with "node$", set device class, device type,
     *  and default buffer size.
     */
    ucb->ucb$r_ucb.ucb$l_devchar2 = DEV$M_NNM;
    ucb->ucb$r_ucb.ucb$b_devclass = DC$_MISC;
    /* ucb->ucb$r_ucb.ucb$b_devtype = LP$_LP11; */  /* we have no device type */
    ucb->ucb$r_ucb.ucb$w_devbufsiz = sizeof(ucb->ucb$f_avgs);   /* 36 byte buffer */
}


/*
 * LR$STRUC_REINIT - Device Data Structure Re-Initialization Routine
 *
 * Functional description:
 *
 *   This routine is called once for each unit by the $LOAD_DRIVER service
 *   immediately after the structure initialization routine is called.
 *
 *   Additionally, this routine is called once for each unit by the $LOAD_DRIVER
 *   service when a driver image is RELOADED.  Thus, this routine is
 *   responsible for filling in the fields in the I/O database structures
 *   that point into this driver image.
 *
 *   Note that this routine must confine its actions to filling in these I/O
 *   database structures.
 *
 * Calling convention:
 *
 *   lax_struc_reinit (crb, ddb, idb, orb, ucb)
 *
 * Input parameters:
 *
 *   crb        Pointer to associated controller request block.
 *   ddb        Pointer to associated device data block.
 *   idb        Pointer to associated interrupt dispatch block.
 *   orb        Pointer to associated object rights block.
 *   ucb        Pointer to the unit control block that is to be initialized.
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   None.
 *
 * Environment:
 * 
 *   Kernel mode, system context, IPL may be as high as 31 and may not be
 *   altered.
 *
 */ 

void lax_struc_reinit (CRB *crb, DDB *ddb, IDB *idb, ORB *orb, LAX_UCB *ucb) {
    
    extern DDT driver$ddt;

    /* Setup the pointer from our DDB in the I/O database to the driver
     * dispatch table that's within this driver image.
     */
    ddb->ddb$ps_ddt = &driver$ddt;

    /* Setup the procedure descriptor and code entry addresses in the VEC
     * portion of the CRB in the I/O database to point to the interrupt
     * service routine that's within this driver image.
     */
    /* dpt_store_isr (crb, lax_interrupt); */
}


/*
 * LAX_UNIT_INIT - Unit Initialization Routine
 *
 * Functional description:
 *
 *   This routine is called once for each unit by the $LOAD_DRIVER service
 *   after a new unit control block has been created, initialized, and
 *   fully integrated into the I/O database.
 *
 *   This routine is also called for each unit during power fail recovery.
 *
 *   It is the responsibility of this routine to bring unit "on line" and
 *   to make it ready to accept I/O requests.
 *
 * Calling convention:
 *
 *   status = lax_unit_init (idb, ucb)
 *
 * Input parameters:
 *
 *   idb        Pointer to associated interrupt dispatch block.
 *   ucb        Pointer to the unit control block that is to be initialized.
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   status     SS$_NORMAL indicates that the unit was initialized successfully.
 *              SS$_IVADDR indicates that an unexpected ISA I/O address or IRQ
 *                         level was detected.
 *
 * Environment:
 * 
 *   Kernel mode, system context, IPL 31.
 */
 
int lax_unit_init (IDB *idb, LAX_UCB *ucb)  {

#if defined DEBUG
    /* If a debug version of this driver is being built then invoke the loaded
     * system debugger.  This could either be the High Level Language System
     * Debugger, XDELTA, or nothing.
     */
    {
        extern void ini$brk (void);
        ini$brk ();
    }
#endif

    /* Set device initially offline (for error exits) and initialize other
     * UCB fields.
     */
    ucb->ucb$r_ucb.ucb$v_online = 0;

    /* Clear the stats array and priority mask. */

    memset(&(ucb->ucb$f_avgs), 0, sizeof(ucb->ucb$f_avgs));
    ucb->ucb$l_priority_mask = 0;

    /* This driver can service only a single unit per DDB and IDB.  Thus,
     * make the single unit the permanent owner of the IDB.  This facilitates
     * getting the UCB address in our interrupt service routine.
     * We don't have an ISR, but if we did, that might be useful.
     */
    idb->idb$ps_owner = &(ucb->ucb$r_ucb); 

    /* Set up the first call to update load averages via fork-wait. */

    fork_wait (lax_update_stats_fork, NULL, NULL, ucb);

    /* Mark the device as online and ready to accept I/O requests */

    ucb->ucb$r_ucb.ucb$v_online = 1;

    return SS$_NORMAL;
}


/*
 * LAX_READ - FDT Routine for Read Function Codes 
 *
 * Functional description:
 *
 *   Verifies the read arguments, then copies as much data as requested.
 *
 *   Since this is an upper-level FDT routine, this routine always returns
 *   the SS$_FDT_COMPL status.  The $QIO status that is to be returned to
 *   the caller of the $QIO system service is returned indirectly by the
 *   FDT completion routines (e. g. exe_std$abortio, exe_std$qiodrvpkt) via
 *   the FDT context structure.
 *
 * Calling convention:
 *
 *   status = lax_read (irp, pcb, ucb, ccb)
 *
 * Input parameters:
 *
 *   irp        Pointer to I/O request packet
 *   pcb        Pointer process control block
 *   ucb        Pointer to unit control block
 *   ccb        Pointer to channel control block
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   status     SS$_FDT_COMPL
 *
 * Environment:
 * 
 *   Kernel mode, user process context, IPL 2.
 */

int lax_read (IRP *irp, PCB *pcb, LAX_UCB *ucb, CCB *ccb) {

    CHAR_PQ qio_bufp;           /* 64-bit pointer to caller's buffer */
    int qio_buflen;             /* Number of bytes in caller's buffer */

    /* Get the pointer to the caller's buffer and the size of the caller's
     * buffer from the $QIO P1 and P2 parameters respectively.  The caller's
     * buffer is treated as a 64-bit address although it may be a 32-bit 
     * address.
     */
    qio_bufp   = (CHAR_PQ)irp->irp$q_qio_p1;

    /* Return an SS$_BADPARAM error if the read size is too small. */
    if (irp->irp$l_qio_p2 < sizeof(float)) {
	return ( call_abortio (irp, pcb, (UCB *)ucb, SS$_BADPARAM) );
    }

    /* Truncate the read size to 36 bytes if needed. */
    if (irp->irp$l_qio_p2 > sizeof(ucb->ucb$f_avgs)) {
	irp->irp$l_qio_p2 = sizeof(ucb->ucb$f_avgs);
    }

    qio_buflen = irp->irp$l_qio_p2;

    /* Assure that the caller has write access to this buffer to do a read
     * operation.  If not, exe_std$readchk will abort the I/O request and
     * return the SS$_FDT_COMPL warning status.  If this is the case, we must
     * return back to the FDT dispatcher in the $QIO system service.
     */
    if (qio_buflen != 0) {
        int status = exe_std$readchk (irp, pcb, &(ucb->ucb$r_ucb), 
                                      qio_bufp, qio_buflen);
        if ( ! $VMS_STATUS_SUCCESS(status) ) return status;

	memcpy( qio_bufp, &(ucb->ucb$f_avgs), qio_buflen );
    }

    return ( call_finishio (irp, (UCB *)ucb, SS$_NORMAL, 0) );
}

/*
 * LAX_WRITE - FDT Routine for Write Function Codes 
 *
 * Functional description:
 *
 *   This is currently commented out because I don't want to reimplement
 *   the original LAVDRIVER behavior that lets you mask out processes at
 *   specific priorities from being counted. I bet nobody has ever used
 *   that feature. Later versions of this driver could add functions here.
 *
 *   Since this is an upper-level FDT routine, this routine always returns
 *   the SS$_FDT_COMPL status.  The $QIO status that is to be returned to
 *   the caller of the $QIO system service is returned indirectly by the
 *   FDT completion routines (e. g. exe_std$abortio, exe_std$qiodrvpkt) via
 *   the FDT context structure.
 *
 * Calling convention:
 *
 *   status = lax_write (irp, pcb, ucb, ccb)
 *
 * Input parameters:
 *
 *   irp        Pointer to I/O request packet
 *   pcb        Pointer process control block
 *   ucb        Pointer to unit control block
 *   ccb        Pointer to channel control block
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   status     SS$_FDT_COMPL
 *
 * Environment:
 * 
 *   Kernel mode, user process context, IPL 2.
 */

#if 0
int lax_write (IRP *irp, PCB *pcb, LAX_UCB *ucb, CCB *ccb) {

    CHAR_PQ qio_bufp;           /* 64-bit pointer to caller's buffer */
    int qio_buflen;             /* Number of bytes in caller's buffer */

    /* Check if the caller has the required CMKRNL privilege. */
    if ((pcb->pcb$q_priv & PRV$M_CMKRNL) == 0)
        return ( call_abortio (irp, pcb, (UCB *)ucb, SS$_NOPRIV) );

    /* Get the pointer to the caller's buffer and the size of the caller's
     * buffer from the $QIO P1 and P2 parameters respectively.  The caller's
     * buffer is treated as a 64-bit address although it may be a 32-bit 
     * address.
     */
    qio_bufp   = (CHAR_PQ)irp->irp$q_qio_p1;

    /* Return an SS$_BADPARAM error if the write size is too small. */
    if (irp->irp$l_qio_p2 < sizeof(uint32_t)) {
	return ( call_abortio (irp, pcb, (UCB *)ucb, SS$_BADPARAM) );
    }

    /* Truncate the write size to 4 bytes if needed. */
    if (irp->irp$l_qio_p2 > sizeof(uint32_t)) {
	irp->irp$l_qio_p2 = sizeof(uint32_t);
    }

    qio_buflen = irp->irp$l_qio_p2;

    /* Assure that the caller has read access to this buffer to do a write
     * operation.  If not, exe_std$writechk will abort the I/O request and
     * return the SS$_FDT_COMPL warning status.  If this is the case, we must
     * return back to the FDT dispatcher in the $QIO system service.
     */
    if (qio_buflen != 0) {
        int status = exe_std$writechk (irp, pcb, &(ucb->ucb$r_ucb), 
                                   qio_bufp, qio_buflen);
        if ( ! $VMS_STATUS_SUCCESS(status) ) return status;

	/* Invert the priority mask for easier use later. */
	ucb->ucb$l_priority_mask = ~(*((__unaligned uint32_t*) qio_bufp));
    }

    return ( call_finishio (irp, (UCB *)ucb, SS$_NORMAL, 0) );
}
#endif


/*
 * LAX_UPDATE_STATS_FORK - Periodic update of load averages
 *
 * Functional description:
 *
 *   This routine performs a once-a-second update of the load average data.
 *   The algorithm is the same as the original LAXDRIVER, except that the
 *   system load average isn't divided by the number of active CPUs, and
 *   the values are all returned as IEEE floats, not VAX F_floating type.
 *
 * Calling convention:
 *
 *   lax_update_stats_fork (irp, not_used, ucb)
 *
 * Input parameters:
 *
 *   not_used	Unused fork routine parameter fr3
 *   not_used   Unused fork routine parameter fr4
 *   ucb        Pointer to unit control block
 *
 * Output parameters:
 *
 *   None.
 *
 * Return value:
 *
 *   None.
 *
 * Environment:
 * 
 *   Kernel mode, system context, fork IPL, fork lock held.
 */

void lax_update_stats_fork (void *fr3, void *fr4, LAX_UCB *ucb) {
    int orig_ipl;
    int status;

    int busy_processes = 0;
    int io_queue_len = 0;

    /* Use $GETRMI to get the process counts asynchronously. */
    // RMI$_COLPG = # of procs in collided page wait state
    // RMI$_COM = # of procs in computable state
    // RMI$_COMO = # of proc in computable outswapped state
    // RMI$_CUR = # of proc currently executing
    // RMI$_FPG = # of proc in free page wait state
    // RMI$_PFW = # of proc in page fault wait state


    /* TODO: fill in stats here */

    /* Acquire the UCB device lock, raise IPL, saving original IPL */
    device_lock (ucb->ucb$r_ucb.ucb$l_dlck, RAISE_IPL, &orig_ipl);

    /* TODO: update stats here */

    /* Restore the UCB device lock, return to the original entry IPL */
    device_unlock (ucb->ucb$r_ucb.ucb$l_dlck, orig_ipl, SMP_RESTORE);

    /* Setup to check the device again in one second via the fork-wait queue */
    fork_wait (lax_update_stats_fork, NULL, NULL, ucb);
}
