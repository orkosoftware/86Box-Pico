/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the PicoMEM / PicoMEM 2 ISA card emulation.
 */
#ifndef EMU_PICOMEM_H
#define EMU_PICOMEM_H

#ifdef __cplusplus
extern "C" {
#endif

extern const device_t picomem_device;
extern const device_t picomem2_device;
extern const device_t picomem_ne2000_device;
extern const device_t picogus_ne2000_device;

extern void *picogus_ne2k_instance(void);
extern void  picogus_ne2k_map(void *priv, uint16_t port, uint8_t irq);

#ifdef __cplusplus
}
#endif

#endif /*EMU_PICOMEM_H*/
