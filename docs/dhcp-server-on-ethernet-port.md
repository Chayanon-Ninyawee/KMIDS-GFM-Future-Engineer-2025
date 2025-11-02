# Setting Up a DHCP Server Using Ethernet Port with Internet Connection from Wireless LAN

This guide will show you how to set up your laptop or computer's ethernet port to act as a DHCP server, providing IP addresses and internet access to a connected device through the ethernet interface, while the laptop receives its internet connection via wireless LAN.

> **Tested on Ubuntu and Fedora.**\
> On Fedora, the package names and service commands differ slightly — see the [Fedora Notes](#fedora-notes) section at the end.

______________________________________________________________________

## Step 1: Install Required Packages

Before starting, ensure you have `isc-dhcp-server` and `iptables` installed. You can install them by running:

### Ubuntu/Debian:

```bash
sudo apt update
sudo apt install isc-dhcp-server iptables-persistent
```

### Fedora:

```bash
sudo dnf install dhcp-server iptables-services
```

______________________________________________________________________

## Step 2: How to Find `YOUR_ETHERNET_INTERFACE` and `YOUR_WIRELESS_INTERFACE`:

To find the name of your interfaces, run the following command:

```bash
ip a
```

Look for the interface name associated with your ethernet connection (it could be something like `enp3s0`, `enp3s0f0`, etc.). Replace all `YOUR_ETHERNET_INTERFACE` in later steps with the name of your ethernet interface.

Then find the name of your wireless interface (it might be `wlp1s0`, `wlan0`, or something similar). Replace all `YOUR_WIRELESS_INTERFACE` in later steps with the name of your wireless interface.

______________________________________________________________________

## Step 3: Configure DHCP Server

### 3.1 Edit DHCP Server Configuration

First, configure the DHCP server to listen on your ethernet interface. Run the following command to edit the DHCP server configuration:

#### Ubuntu:

```bash
sudo nano /etc/default/isc-dhcp-server
```

Modify the file to look like this:

```bash
INTERFACESv4="YOUR_ETHERNET_INTERFACE"
INTERFACESv6=""
```

#### Fedora:

Don't need to do anything.

______________________________________________________________________

### 3.2 Define IP Range and Subnet in DHCP Config

Next, define the IP range and subnet to be used by the DHCP server:

```bash
sudo nano /etc/dhcp/dhcpd.conf
```

Add the following configuration:

```bash
subnet 192.168.39.0 netmask 255.255.255.0 {
  range 192.168.39.10 192.168.39.50;
  option subnet-mask 255.255.255.0;
  option routers 192.168.39.1;
}
```

This configuration will provide IP addresses in the range of `192.168.39.10` to `192.168.39.50`.

______________________________________________________________________

## Step 4: Configure Ethernet Interface for Static IP

### 4.1 Add a New Network Connection

Now, you need to add a static IP address to your ethernet interface and configure it to use the DHCP server. Run the following `nmcli` command:

```bash
nmcli connection add type ethernet con-name "DHCP Server" ifname YOUR_ETHERNET_INTERFACE autoconnect yes \
  connection.autoconnect-priority 0 ipv4.method manual ipv4.addresses 192.168.39.1/24 \
  ipv4.route-metric -1 ipv4.may-fail yes ipv4.dhcp-send-hostname yes ipv6.method disabled
```

Replace `YOUR_ETHERNET_INTERFACE` with the interface name you found earlier.

This assigns the IP `192.168.39.1` to the ethernet interface.

______________________________________________________________________

## Step 5: Enable IP Forwarding

### 5.1 Enable IP Forwarding in `sysctl.conf`

#### Ubuntu:

To forward internet traffic from your wireless LAN to the ethernet interface, enable IP forwarding by running:

```bash
sudo nano /etc/sysctl.conf
```

Uncomment the line that enables IP forwarding:

```bash
#net.ipv4.ip_forward=1
```

Remove the `#` so it becomes:

```bash
net.ipv4.ip_forward=1
```

#### Fedora:

On Fedora, the recommended way is to create a custom config file:

```bash
echo "net.ipv4.ip_forward=1" | sudo tee /etc/sysctl.d/99-ipforward.conf
```

### 5.2 Apply Changes

#### Ubuntu:

Apply the changes to the system settings:

```bash
sudo sysctl -p
```

#### Fedora:

Apply the changes to the system settings:

```bash
sudo sysctl --system
```

______________________________________________________________________

## Step 6: Set Up IP Masquerading

### 6.1 Configure iptables for NAT

Now, use `iptables` to configure Network Address Translation (NAT) so the connected device can use the internet from your wireless connection. Run the following commands:

```bash
sudo iptables -t nat -A POSTROUTING -o YOUR_WIRELESS_INTERFACE -j MASQUERADE
sudo iptables -A FORWARD -i YOUR_ETHERNET_INTERFACE -o YOUR_WIRELESS_INTERFACE -j ACCEPT
sudo iptables -A FORWARD -i YOUR_WIRELESS_INTERFACE -o YOUR_ETHERNET_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
```

For `YOUR_ETHERNET_INTERFACE` and `YOUR_WIRELESS_INTERFACE`, use the interface names you found earlier.

______________________________________________________________________

### 6.2 Save iptables Rules

#### Ubuntu:

```bash
sudo netfilter-persistent save
```

#### Fedora:

```bash
sudo service iptables save
sudo systemctl enable iptables
sudo systemctl start iptables
```

______________________________________________________________________

## Step 7: Restart Services

### 7.1 Restart DHCP Server

#### Ubuntu:

```bash
sudo systemctl restart isc-dhcp-server
```

#### Fedora:

```bash
sudo systemctl restart dhcpd
```

Your laptop's ethernet port is now acting as a DHCP server, providing IP addresses to connected devices and routing internet traffic from your wireless LAN.

______________________________________________________________________

## Fedora Notes

- Fedora uses **`dhcpd`** instead of `isc-dhcp-server`.
- Configuration files are in:
  - `/etc/sysconfig/dhcpd`
  - `/etc/dhcp/dhcpd.conf`
- Service name: `dhcpd`
- iptables rules are managed via `iptables-services`.
- If `.local` hostnames (like `rpi5.local`) stop resolving after switching from Wi-Fi to Ethernet, restart Avahi:
  ```bash
  sudo systemctl restart avahi-daemon
  ```
  or force rescan:
  ```bash
  sudo avahi-daemon --kill && sudo avahi-daemon --daemonize
  ```

______________________________________________________________________

## Additional Tip: Prevent Ethernet Power-Down on Battery

If you notice that the ethernet interface (e.g., `enp3s0f0`) doesn't wake up properly when reconnecting the cable, especially on battery power, it may be due to power management putting the NIC into a low-power state.

You can apply this temporary fix to force it to stay on:

```bash
echo on | sudo tee /sys/class/net/YOUR_ETHERNET_INTERFACE/device/power/control
```

This disables the power-saving mode for that ethernet device until the next reboot. Replace `YOUR_ETHERNET_INTERFACE` with your actual ethernet interface.
