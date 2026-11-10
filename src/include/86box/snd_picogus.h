/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PicoGUS multi-mode ISA sound card.
 */
#ifndef SOUND_SND_PICOGUS_H
#define SOUND_SND_PICOGUS_H

extern const device_t picogus_device;
extern const device_t mouse_picogus_device;

extern int mouse_picogus_selected(void);

#endif /* SOUND_SND_PICOGUS_H */
