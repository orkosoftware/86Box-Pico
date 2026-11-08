/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Gravis UltraSound core for cards that embed one and switch it
 *          on and off at run time, such as the PicoMEM.
 */
#ifndef SOUND_SND_GUS_H
#define SOUND_SND_GUS_H

extern void *gus_embedded_init(void);
extern void  gus_embedded_set_port(void *priv, uint16_t port);
extern void  gus_embedded_set_irq_dma(void *priv, int irq, int dma);
extern void  gus_get_buffer(int32_t *buffer, uint16_t len, void *priv);
extern void  gus_close(void *priv);
extern void  gus_speed_changed(void *priv);

#endif /* SOUND_SND_GUS_H */
