# How to Image Drive in Linux

To create an image of a drive in Linux, you'll use the `dd` command. Below is a simple guide to calculate the count and perform the imaging.

## 1. Calculate Count

The `count` parameter defines the number of blocks to be copied. To calculate it:

```
count = <size to image> / <block size>
```

For example:
```
count = 6GB / 4MB = 1500
```

In this case, you are copying 6GB of data with a block size of 4MB, which gives a count of 1500.

## 2. Check Disk Space with `df -h`

Before performing any disk imaging, it's a good idea to check the available space on your system and the target drive. You can do this with the `df -h` command:

```
df -h
```

### Explanation:
- `df`: Displays information about disk space usage.
- `-h`: Makes the output human-readable (e.g., showing sizes in GB, MB).
  
This command will show the available space on all mounted filesystems. Make sure your destination drive has enough space to accommodate the image file.

### **Note:** Before using `dd` to copy or restore an image, always verify that the correct drive is selected, as selecting the wrong drive could lead to data loss. You can double-check the device path using `lsblk` or `fdisk -l`.

## 3. To Image a Drive

**Important Note:** Before running the `dd` command, always double-check the `if` and `of` parameters before running the `dd` command to avoid overwriting important data. Double-check with `lsblk` or `fdisk -l` to confirm the device paths.

Use the following command to create an image of the drive:

```
sudo dd if=<your_drive> of=<your_image_file> bs=<your_block_size> count=<your_count> status=progress && sync
```

### Explanation of the Command:
- `if=<your_drive>`: Specifies the input file (your source drive, e.g., `/dev/sda`).
- `of=<your_image_file>`: Specifies the output file (destination for the image, e.g., `/home/garfieldcmix/Desktop/ImageFiles/gfmrpi_gfm-rpi4.img`).
- `bs=<your_block_size>`: Specifies the block size (e.g., `4M` for 4MB).
- `count=<your_count>`: Specifies the number of blocks to copy, which you calculated earlier (e.g., `1500`).
- `status=progress`: Displays progress during the operation.
- `&& sync`: Ensures all data is written to the disk before finishing.

## 4. To Restore the Image to a Drive

**Important Note:** Restoring an image to a drive will overwrite all data on the target drive. Before running the `dd` command, always double-check the `if` and `of` parameters before running the `dd` command to avoid overwriting important data. Double-check with `lsblk` or `fdisk -l` to confirm the device paths.

To restore the image back to a drive, use the following command:

```
sudo dd if=<your_image_file> of=<your_drive> bs=<your_block_size> status=progress && sync
```

### Explanation of the Command:
- `if=<your_image_file>`: Specifies the input image file (e.g., `/home/garfieldcmix/Desktop/ImageFiles/gfmrpi_gfm-rpi4.img`).
- `of=<your_drive>`: Specifies the output drive (e.g., `/dev/sda`).
- `bs=<your_block_size>`: Specifies the block size (e.g., `4M` for 4MB).
- `status=progress`: Displays progress during the operation.
- `&& sync`: Ensures all data is written to the disk before finishing.

## 5. Alternative Method: Using Disk Imager in Linux Mint

1. Open **Disk Image Writer** in Linux Mint.
2. Click the "..." on the top right and select **Restore Disk Image**.
3. Select the image file you want to write.
4. Select the target device.
5. Click "Restore" to begin the process.

This method provides a graphical interface and is simpler if you're not comfortable using the terminal.
