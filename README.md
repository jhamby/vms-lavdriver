# vms-laxdriver
[OpenVMS](https://vmssoftware.com/) Load Average eXtended
device driver (LAX0:), written in C, using IEEE floats.

This driver is designed after the original LAVDRIVER, provided for reference
in the `orig-src` directory. The major differences are that it's now much
smaller, because it's written in C instead of VAX MACRO, it returns the
values as IEEE floats instead of VAX F_floating values (you can compile the
test program with different `/FLOAT` values to verify that they match), and
the system load average computation now gets values from the system instead
of locking the *SCHED* spinlock and counting them manually.

Another difference from LAVDRIVER is that the load averages are not divided
by the number of active CPUs, but reflect the total number of processes
actively running or ready to run. This matches UNIX `getloadavg()` behavior.

Loading the driver works the same way as LAVDRIVER. From a DCL script:

```
$ Run SYS$SYSTEM:SYSMAN
IO Load LAXDRIVER
IO Connect LAX0 /NoAdapter/Driver=LAXDRIVER
$exit
```

Be sure to specify the path to LAXDRIVER if you haven't copied it to
`SYS$COMMON:[SYS$LDR]`. Use the same commands to load it interactively,
but without typing the "$" (you **do** need to keep the "$" where I've placed
them if you paste it into a DCL startup script).
