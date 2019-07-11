#!/usr/bin/env python3

import os
import difflib
import json
import yaml


data = {}

path = "/Users/captain/openwrt/target/linux/ath79"
device_path = "/Users/captain/openwrt-devices/_data/devices/"
device_file_paths = []

vendors = [
    "3Com",
    "4G Systems",
    "7Links",
    "8devices",
    "A-Link",
    "Abicom",
    "ADB",
    "ADI Engineering",
    "ARC Flex",
    "AVM",
    "AXIMCom",
    "Abicom International",
    "Accton",
    "AcmeSystems",
    "Actiontec",
    "Aerohive",
    "Afoundry",
    "Agestar",
    "Aigale",
    "AirTight",
    "Airlink101",
    "ALFA Network" "Akitio",
    "Alcatel-Sbell",
    "Allnet",
    "Alpha Networks",
    "ALFA Network",
    "Arcadyan",
    "Arcadyan / Astoria",
    "Arduino.cc",
    "Aruba",
    "AsiaRF",
    "Asmax",
    "Asus",
    "Atlantis",
    "Atmel",
    "AudioCodes",
    "Avnet",
    "Aztech",
    "BDCOM",
    "BT",
    "BT Openreach",
    "Baidu",
    "Belkin",
    "Billion",
    "Bitmain",
    "Blueendless",
    "Buffalo",
    "COMFAST",
    "Catch Tec",
    "Cisco",
    "Compex",
    "CompuLab",
    "Comtrend",
    "Creator",
    "Cubitech",
    "Cudy",
    "D-Link",
    "D-Team",
    "Davolink",
    "Devolo",
    "Digilent",
    "Digineo",
    "DomyWifi",
    "Dovado",
    "Dragino",
    "DuZun",
    "ELECOM",
    "EasyAcc",
    "EasyLink",
    "Edimax",
    "Embedded Wireless",
    "EnGenius",
    "Evaluation boards / unbranded boards",
    "Firefly",
    "Fon",
    "Freecom",
    "Freescale i.MX23/i.MX28",
    "FriendlyElec",
    "Frys",
    "GL.iNet",
    "Gainstrong",
    "Gateway",
    "Gateworks",
    "Gigaset",
    "Globalscale",
    "GnuBee",
    "HAK5",
    "HAME",
    "Head Weblink",
    "Hi-Link",
    "HiWiFi/Gee",
    "Hitachi",
    "Hnet",
    "HooToo",
    "Huawei",
    "I-O Data",
    "ITian",
    "Intellidesign",
    "Inteno",
    "Intenso",
    "Inventel",
    "Iomega",
    "JCG",
    "Kingston",
    "Kintec",
    "Laird",
    "Lamobo",
    "Lava",
    "Lemaker",
    "Lenovo",
    "Librerouter",
    "Linksprite",
    "Linksys",
    "Loewe",
    "Logitech",
    "MQMaker",
    "MTC",
    "Marvell",
    "MeLE",
    "MediaTek",
    "Medion",
    "Meraki",
    "Mercury",
    "Microduino",
    "MikroTik",
    "Multilaser",
    "NEC",
    "NXP",
    "NC-Link",
    "NetComm",
    "Netgear",
    "Netis",
    "Nexx",
    "NixCore",
    "NuCom",
    "OMYlink",
    "Observa",
    "Ocedo",
    "Olimex",
    "Omnima",
    "On Networks",
    "Onion",
    "Open-Mesh",
    "OpenEmbed",
    "Option",
    "Oyewifi",
    "P&W",
    "PC Engines",
    "PHICOMM",
    "PISEN",
    "PQI",
    "Pacific Networks",
    "Patriot Memory",
    "Petatel",
    "Pine64",
    "Pirelli",
    "Planex",
    "PlatHome",
    "Pogoplug",
    "Poray",
    "PowerCloud Systems",
    "Prolink",
    "Qualcomm Atheros",
    "Atheros",
    "QEMU (aarch64)",
    "QEMU (armvirt)",
    "QEMU (i386)",
    "QEMU (malta)",
    "QEMU (x86_64)",
    "Qi hardware",
    "Qihoo hardware",
    "Qxwlan",
    "RaidSonic",
    "Rakwireless",
    "Raspberry Pi Foundation",
    "RavPower",
    "Redwave",
    "Rosewill",
    "SFR",
    "SMC",
    "STORYLiNK",
    "Sagem",
    "SamKnows",
    "Samsung",
    "Sanlinking",
    "Scientific Atlanta",
    "Seagate",
    "Senao",
    "Sercomm",
    "Shuttle",
    "Siemens",
    "SimpleTech",
    "Sinovoip",
    "Sitecom",
    "Sky",
    "Skylab",
    "Skyline",
    "SolidRun",
    "Sophos",
    "Sparklan",
    "Strong",
    "T-Com / Telekom",
    "TOTOLINK",
    "TP-Link",
    "TRENDnet",
    "Tama",
    "Technicolor",
    "Tecom",
    "Telco Electronics",
    "Telsey",
    "Teltonika",
    "Telldus",
    "Tenda",
    "Texas Instruments",
    "Thomson",
    "Toradex",
    "Turris CZ.NIC",
    "US Robotics",
    "Ubiquiti",
    "UniElec",
    "Upvel",
    "VMware",
    "VONETS",
    "Vizio",
    "VoCore",
    "WAVLINK",
    "WIZnet",
    "WRTnode",
    "Wallys",
    "Wansview",
    "WeVO",
    "Western Digital",
    "Widemac",
    "Widora",
    "Wiligear",
    "Winchannel",
    "XILINX",
    "Xiaomi",
    "Xunlong",
    "YOUKU",
    "YouHua",
    "YunCore",
    "ZBT",
    "ZTE",
    "Zcomax",
    "Zlmnet",
    "Zorlik",
    "ZyXEL",
    "eTactica",
    "i.onik",
    "jjPlus",
    "unbranded",
]

revisionPrefixesToCheck = ["rev. ", "v", "V", "("]

files = []


# converts the array to its lower case for better matching chances
lower_vendors = [x.lower() for x in vendors]

# finds index of nth occurence of a substring in a string
def find_nth(string, substring, n):
    i = 0
    while n >= 0:
        n -= 1
        i = string.find(substring, i + 1)
    return i


# finds the most probable YAML file for the device from the openwrt-devices repository
# employs binary search to find out the nearest string
def get_closest_file_name(device_id):
    s = 0
    e = len(device_file_paths)
    mid = (s + e) // 2
    l = len(device_id)
    i = 0
    while e > s:
        if i == l:
            return device_file_paths[mid]
        mid = (s + e) // 2
        a = device_file_paths[mid][i]
        b = device_id[i]
        if a == b:
            i += 1
        elif a > b:
            e = mid
            i = 0
        else:
            s = mid + 1
            i = 0
    return ""


# finds the vendor string from the available array
def get_vendor(device_title):
    # if the DEVICE_TITLE only has the DEVICE_MODEL and no Vendor name
    if device_title.count(" ") == 0:
        return ""
    # converts to lowercase for better chances of match
    device_title = device_title.lower()

    vendor = ""

    # first try to find the vendor name assuming the DEVICE_VENDOR has two words in it
    try:
        # take the first two words
        prefix_vendor = device_title.split(" ")[0] + " " + device_title.split(" ")[1]
        # get close match in the vendors array
        vendor = difflib.get_close_matches(prefix_vendor, lower_vendors)[0]
        # check if the first char of the Vendor's Name matches the frist char of the DEVICE_TITLE
        # A scrappy way of knowing if the `vendor` var has the correct vendor name
        if vendor[0] == device_title[0]:
            # finds the index in the lowered array
            vendor_index = lower_vendors.index(vendor)
            # finalizes the selected vendor
            vendor = vendors[vendor_index]
        else:
            # go to except = maybe the vendor has only one word
            raise ValueError
    except:
        # reached if ValueError is raied or there is no match found in vendor array
        try:
            # take the first word
            prefix_vendor = device_title.split(" ")[0]
            # get close match in the vendors array and repeats the above
            vendor = difflib.get_close_matches(prefix_vendor, lower_vendors)[0]
            if vendor[0] == device_title[0]:
                vendor_index = lower_vendors.index(vendor)
                vendor = vendors[vendor_index]
            else:
                # if the first char of vendor doesn't match that of device_title then we can assume/conclude that
                # there is no vendor
                vendor = ""
        except:
            # reaches if no match is found in the vendors array
            try:
                # tries to find vendor if there is only a substring of Vendor's name in DEVICE_TITLE
                # takes the first word from every vendor's name and tries to find a match
                vendor = difflib.get_close_matches(
                    prefix_vendor, [x.split(" ")[0] for x in lower_vendors]
                )[0]
                if vendor[0] == device_title[0]:
                    vendor_index = lower_vendors.index(vendor)
                    vendor = vendors[vendor_index]
                else:
                    vendor = ""
            except:
                # if nothing is found then pass and return the empty vendor
                pass
    return vendor


# finds the index where revision starts to appear. Used my get_revision
def get_revision_index(device_title, num_vendor):
    prefix_index = 0
    if len(device_title) > num_vendor + 1:
        for prefix in revisionPrefixesToCheck:
            try:
                possible_prefix_index = device_title.index(prefix)
                if device_title[possible_prefix_index - 1] == " ":
                    if prefix_index == 0:
                        prefix_index = possible_prefix_index
                    if possible_prefix_index < prefix_index:
                        prefix_index = possible_prefix_index
            except:
                pass
    #  edge case assuming that if the device name consist of two words, second one will not start with revision-prefix
    second_space_index = find_nth(device_title, " ", num_vendor)
    return prefix_index if second_space_index < prefix_index else 0


# finds the revision substring from DEVICE_TITLE
def get_revision(device_title, num_vendor):
    prefix_index = get_revision_index(device_title, num_vendor)
    if prefix_index is not 0:
        revision = device_title[prefix_index:]
        # strip paranthesis in case of "(__)" but not in case of revision being "__ (__)"
        if revision[0] == "(":
            revision = revision[1:]
            if revision[-1] == ")":
                revision = revision[:-1]
        return revision
    return ""


# finds DEVICE_MODEL from the DEVICE_TITLE
def get_device_name(device_title, vendor_num):
    prefix_index = get_revision_index(device_title, vendor_num)
    if prefix_index is not 0:
        device_title = device_title[0 : prefix_index - 1]
    device_num = device_title.count(" ") + 1
    # assuming that the device_title will have the same number of words as in Vendor's actual name
    if device_num > vendor_num:
        device_index = (
            find_nth(device_title, " ", vendor_num - 1) + 1 if vendor_num > 0 else 0
        )
        return device_title[device_index:]
    elif device_num == vendor_num:
        return device_title
    else:
        device_index = (
            find_nth(device_title, " ", device_num - 1) + 1 if vendor_num > 0 else 0
        )
        return device_title[device_index:]
    return device_title


def main():
    global device_file_paths
    for r, _, f in os.walk(path):
        for file in f:
            if ".mk" in file:
                files.append(os.path.join(r, file))

    for r, _, f in os.walk(device_path):
        for file in f:
            if ".yml" in file:
                device_file_paths.append(file[:-4])
                device_file_paths = sorted(device_file_paths)

    for filepath in files:
        f = open(filepath, "r")
        lines = f.readlines()
        open(filepath, "w").close()
        f = open(filepath, "a")
        j = 0
        while j < len(lines):
            try:
                line = lines[j]
                if line.count("DEVICE_TITLE") > 0:
                    i = line.index("DEVICE_TITLE")
                    device_title = line[i + 16 : -1]
                    vendor = get_vendor(device_title)
                    vendor_num = vendor.count(" ") + 1 if len(vendor) > 0 else 0
                    revision = get_revision(device_title, vendor_num)
                    device_name = get_device_name(device_title, vendor_num)
                    del lines[j]
                    k = 0
                    if vendor is not "":
                        lines.insert(j + k, "  DEVICE_VENDOR := " + vendor + "\n")
                        k += 1
                    lines.insert(j + k, "  DEVICE_MODEL := " + device_name + "\n")
                    k += 1
                    if revision is not "":
                        lines.insert(j + k, "  DEVICE_VARIANT := " + revision + "\n")
                        k += 1
                    try:
                        device_id = (
                            (
                                vendor.lower().replace(" ", "_").replace(".", "-") + "_"
                                if vendor != ""
                                else ""
                            )
                            + device_name.lower().replace(" ", "_").split("/")[0]
                            + (
                                ("_" + revision.lower().split("/")[0].split(" ")[0])
                                if revision != ""
                                else ""
                            )
                        )
                        possible_file_name = get_closest_file_name(device_id)
                        possible_file_name = (
                            possible_file_name
                            if possible_file_name != ""
                            else difflib.get_close_matches(
                                device_id, device_file_paths
                            )[0]
                        )
                        device_data_path = device_path + possible_file_name + ".yml"
                        #try:
                        #    with open(device_data_path, "r") as fh:
                        #        device_data = yaml.safe_load(fh)
                        #        try:
                        #            ram = device_data["ram_mb"]
                        #            lines.insert(
                        #                j + k, "  DEVICE_RAM := " + str(ram) + "\n"
                        #            )
                        #            k += 1
                        #        except:
                        #            pass
                        #        try:
                        #            flash = device_data["flash_mb"]
                        #            lines.insert(
                        #                j + k, "  DEVICE_FLASH := " + str(flash) + "\n"
                        #            )
                        #            k += 1
                        #        except:
                        #            pass
                        #except FileNotFoundError:
                            # Keep preset values
                            print(device_id)
                            pass
                    except Exception as e:
                        print(e)
                        print(device_id)
                        pass
            except Exception as e:
                print(e)
                pass
            j = j + 1
        f.writelines(lines)


main()
