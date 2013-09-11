/*
 * The Foot Switch Driver
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb/input.h>
#include <linux/hid.h>

#define FSD_VENDOR_ID 0x0426
#define FSD_PRODUCT_ID 0x3011

/*
 * Version Information
 */
#define DRIVER_VERSION "1.0"
#define DRIVER_AUTHOR "Dorian Zaccaria <dorian.zaccaria@gmail.com>"
#define DRIVER_DESC "USB HID Boot Protocol foot switch keyboard driver"
#define DRIVER_LICENSE "GPL"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

static const unsigned char      usb_fsd_keycode[3] = {14, 28, 152};

/*
 * Our private structure
 */
struct                          usb_fsd {
    struct input_dev            *dev;
    struct usb_device           *usbdev;
    unsigned char               old[8];
    struct urb                  *irq;
    char                        name[128];
    char                        phys[64];
    unsigned char               *new;
    dma_addr_t                  new_dma;
};

/*
 * The IRQ handler
 */
static void                     usb_fsd_irq(struct urb                          *urb)
{
  struct usb_fsd                *fsd = urb->context;
  int                           i;

  switch (urb->status) {
    case 0:
      break;
    case -ECONNRESET:
    case -ENOENT:
    case -ESHUTDOWN:
      return;
    default:
      goto resubmit;
  }

  for (i = 2; i < 8; i++)
  {
    if (fsd->old[i] > 3 && memscan(fsd->new + 2, fsd->old[i], 6) == fsd->new + 8)
    {
      if (usb_fsd_keycode[fsd->old[i] - 30])
        input_report_key(fsd->dev, usb_fsd_keycode[fsd->old[i] - 30], 0);
      else
        hid_info(urb->dev, "Unknown key (scancode %#x) released.\n",
                 fsd->old[i]);
    }

    if (fsd->new[i] > 3 && memscan(fsd->old + 2, fsd->new[i], 6) == fsd->old + 8)
    {
      if (usb_fsd_keycode[fsd->new[i] - 30])
        input_report_key(fsd->dev, usb_fsd_keycode[fsd->new[i] - 30], 1);
      else
        hid_info(urb->dev, "Unknown key (scancode %#x) released.\n",
                 fsd->new[i]);
    }
  }

  input_sync(fsd->dev);

  memcpy(fsd->old, fsd->new, 8);

  resubmit:
  i = usb_submit_urb (urb, GFP_ATOMIC);
  if (i)
    hid_err(urb->dev, "can't resubmit intr, %s-%s/input0, status %d",
            fsd->usbdev->bus->bus_name,
            fsd->usbdev->devpath, i);
}

/*
 * The open function, enable the irq (open the syscall urb)
 */
static int                      usb_fsd_open(struct input_dev                   *dev)
{
  struct usb_fsd                *fsd = input_get_drvdata(dev);

  fsd->irq->dev = fsd->usbdev;
  if (usb_submit_urb(fsd->irq, GFP_KERNEL))
    return -EIO;

  return 0;
}

/*
 * This function close the urb
 */
static void                     usb_fsd_close(struct input_dev                  *dev)
{
  struct usb_fsd                *fsd = input_get_drvdata(dev);

  usb_kill_urb(fsd->irq);
}

/*
 * This function allocate ressources for urb
 */
static int                      usb_fsd_alloc_mem(struct usb_device             *dev,
                                                  struct usb_fsd                *fsd)
{
  if (!(fsd->irq = usb_alloc_urb(0, GFP_KERNEL)))
    return -1;
  if (!(fsd->new = usb_alloc_coherent(dev, 8, GFP_ATOMIC, &fsd->new_dma)))
    return -1;
  return 0;
}

/*
 * This function free ressources
 */
static void                     usb_fsd_free_mem(struct usb_device              *dev,
                                                 struct usb_fsd                 *fsd)
{
  usb_free_urb(fsd->irq);
  usb_free_coherent(dev, 8, fsd->new, fsd->new_dma);
}

/*
 * The probe function
 */
static int                      usb_fsd_probe(struct usb_interface              *iface,
                                              const struct usb_device_id        *id)
{

  struct usb_device             *dev = interface_to_usbdev(iface);
  struct usb_host_interface     *interface;
  struct usb_endpoint_descriptor *endpoint;
  struct usb_fsd                *fsd;
  struct input_dev              *input_dev;
  int                           i, pipe, maxp;
  int                           error = -ENOMEM;

  interface = iface->cur_altsetting;

  if (interface->desc.bNumEndpoints != 1)
    return -ENODEV;

  endpoint = &interface->endpoint[0].desc;
  if (!usb_endpoint_is_int_in(endpoint))
    return -ENODEV;

  pipe = usb_rcvintpipe(dev, endpoint->bEndpointAddress);
  maxp = usb_maxpacket(dev, pipe, usb_pipeout(pipe));

  fsd = kzalloc(sizeof(struct usb_fsd), GFP_KERNEL);

  input_dev = input_allocate_device();
  if (!fsd || !input_dev)
    goto fail1;

  if (usb_fsd_alloc_mem(dev, fsd))
    goto fail2;

  fsd->usbdev = dev;
  fsd->dev = input_dev;

  if (dev->manufacturer)
    strlcpy(fsd->name, dev->manufacturer, sizeof(fsd->name));

  if (dev->product) {
    if (dev->manufacturer)
      strlcat(fsd->name, " ", sizeof(fsd->name));
    strlcat(fsd->name, dev->product, sizeof(fsd->name));
  }

  if (!strlen(fsd->name))
    snprintf(fsd->name, sizeof(fsd->name),
             "USB Foot Switch Keyboard %04x:%04x",
             le16_to_cpu(dev->descriptor.idVendor),
             le16_to_cpu(dev->descriptor.idProduct));

  usb_make_path(dev, fsd->phys, sizeof(fsd->phys));
  strlcat(fsd->phys, "/input0", sizeof(fsd->phys));

  input_dev->name = fsd->name;
  input_dev->phys = fsd->phys;
  usb_to_input_id(dev, &input_dev->id);
  input_dev->dev.parent = &iface->dev;

  input_set_drvdata(input_dev, fsd);

  input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

  for (i = 0; i < 255; i++)
    set_bit(usb_fsd_keycode[i], input_dev->keybit);
  clear_bit(0, input_dev->keybit);

  input_dev->open = usb_fsd_open;
  input_dev->close = usb_fsd_close;

  usb_fill_int_urb(fsd->irq, dev, pipe,
                   fsd->new, (maxp > 8 ? 8 : maxp),
                   usb_fsd_irq, fsd, endpoint->bInterval);
  fsd->irq->transfer_dma = fsd->new_dma;
  fsd->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

  error = input_register_device(fsd->dev);
  if (error)
    goto fail2;

  usb_set_intfdata(iface, fsd);
  printk(KERN_INFO "My USB Keyboard: %s is up", fsd->name);
  return 0;

  fail2:
  usb_fsd_free_mem(dev, fsd);
  fail1:
  input_free_device(input_dev);
  kfree(fsd);
  return error;
}

/*
 * If the device is disconnected, this function will free all resources
 */
static void                     usb_fsd_disconnect(struct usb_interface         *intf)
{
  struct usb_fsd                *fsd = usb_get_intfdata(intf);

  usb_set_intfdata(intf, NULL);

  if (fsd)
  {
    usb_kill_urb(fsd->irq);
    input_unregister_device(fsd->dev);
    usb_fsd_free_mem(interface_to_usbdev(intf), fsd);
    kfree(fsd);
  }
}


static struct usb_device_id     usb_fsd_id_table [] = {
  { USB_DEVICE(FSD_VENDOR_ID, FSD_PRODUCT_ID) },
  { }
};

static struct usb_driver        usb_fsd_driver = {
  .name =         "usbfsd",
  .probe =        usb_fsd_probe,
  .disconnect =   usb_fsd_disconnect,
  .id_table =     usb_fsd_id_table,
};

/*
 * The initial function which register the usb device
 */
static int __init               usb_fsd_init(void)
{
  int                           ret;

  ret = usb_register(&usb_fsd_driver);
  if (ret == 0)
    printk("usbfsd registered");

  return ret;
}

/*
 * The exit function, deregistered the usb device
 */
static void __exit              usb_fsd_exit(void)
{
  usb_deregister(&usb_fsd_driver);
  printk("usbfsd deregistered");
}

module_init(usb_fsd_init);
module_exit(usb_fsd_exit);
