#!/usr/bin/env python3

#################################################################################
# Copyright (c) 2018 Creative Sphere Limited.
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Apache License v2.0
# which accompanies this distribution, and is available at
# https://www.apache.org/licenses/LICENSE-2.0
#
#  Contributors:
#    Creative Sphere - initial API and implementation
#
#################################################################################

import time
import sys
import os

import nRF2401

DATA_SIZE = 16
PACKET_SIZE = DATA_SIZE + 1

args = sys.argv
scriptFile = args[0]
del args[0]


verbose = 0
timeout = 10
channel = 1
ce_gpio_pin = 25
spi_bus = 0
ADDRESS_AS_STRING = "BOOTL"


def init():
    nRF2401.initNRF(0, spi_bus, PACKET_SIZE, address, channel)


def printHelp(rc):
    print("")
    print("usage: nrf-fwt <common-switches> <command>")
    print("")
    print("    -h or --help                   Help message.")
    print("    -t or --timeout <timeout>      Sets nRF24L01 transmit/receive timeout.")
    print("    -a or --address <address>      Sets nRF24L01 transmit/receive address.")
    print("                                   Address may be in 0xHHHHHHHHHH format or")
    print("                                   just 5 letter ascii string. Default is HOTEL.")
    print("    -c or --channel <channel>      Channel for nRF24L01. Default is 108.")
    print("    -s or --_spi <bus>              _spi bus 0 or 1. Default 0.")
    print("    -e or --ce <bcm-gpio-pin>      CE pin. Default 25.")
    print("")
    print("Commands:")
    print("    write <switches> <file.hex>    Uploads hex file to flash or eeprom.")
    print("    -p or --page <page-size>       Page size. Mandatory if not eeprom. Mandatory for flash.")
    print("    -s or --start                  Reset and start program after upload. Default.")
    print("    -n or --dont-start             Do not reset and start program after upload.")
    print("    -t or --verify                 Verify written data by reading it back. Default.")
    print("    -N or --dont-verify            Do not verify written data.")
    print("    -f or --flash                  Upload hex file to flash. Default.")
    print("    -e or --eeprom                 Upload hex file to eeprom")
    print("")
    print("    verify <file.hex>              Verifies hex file.")
    print("    -p or --page <page-size>       Page size. Mandatory if not eeprom. Mandatory for flash.")
    print("    -f or --flash                  Upload hex file to flash. Default.")
    print("    -e or --eeprom                 Upload hex file to eeprom")
    print("")
    print("    read <switches> <start:end>    Reads flash or eeprom and displays as hex.")
    print("    -f or --flash                  Upload hex file to flash. Default.")
    print("    -e or --eeprom                 Upload hex file to eeprom")
    print("    -w or --wide                   Long format of 64 bytes per row.")
    print("    -l or --long                   Long format of 256 bytes per row.")
    print("")
    print("    version                        Reads version.")
    print("")
    print("    signature                      Reads signature.")
    print("")
    print("    readid                         Reads bootloaders signature.")
    print("")
    print("    reset                          Resets and starts program.")
    print("")
    print("Examples:")
    print("")
    print("  Upload using channel 109, address 0x1212121215 for ATmega328p (128 bytes per page")
    print("     nrf-fwt -c 109 -a 0x1212121215 write -p 128 file.hex")
    print("")
    print("  Upload to eeprom")
    print("     nrf-fwt write -e eeprom.hex")
    print("")
    print("  Upload to and don't start for ATtiny85")
    print("     nrf-fwt write -n -p 64 program.hex")
    print("")
    print("  Read eeprom from 16th to 31st byte")
    print("     nrf-fwt read -e <0x10:0x20>")
    print("")
    sys.exit(rc)


def processAddress(addr):
    if addr.startswith("0x"):
        if len(addr) != 12:
            print("Error: Len of address must be 12 characters long in format of 0xHHHHHHHHHH!")
            printHelp(1)
        else:
            ret = []
            addr = addr[2:]
            for i in range(0, 5):
                ret.append(int("0x" + addr[i * 2: i * 2 + 2], 0))
    elif len(addr) != 5:
        print("Error: Len of address must be 5 characters long!")
        printHelp(1)
    else:
        ret = []
        for i in range(0, 5):
            ret.append(ord(addr[i]))
        return ret


address = processAddress(ADDRESS_AS_STRING)

while len(args) > 0 and args[0].startswith("-"):
    if args[0] == "-h" or args[0] == "--help":
        printHelp(0)

    if args[0] == "-a" or args[0] == "--address":
        if len(args) < 2:
            print("Error: missing address value")
            printHelp(1)
        del args[0]
        address = processAddress(args[0])
        del args[0]
    elif args[0] == "-c" or args[0] == "--channel":
        if len(args) < 2:
            print("Error: missing channel value")
            printHelp(1)
        del args[0]
        channel = int(args[0])
        del args[0]
    elif args[0] == "-s" or args[0] == "--_spi":
        if len(args) < 2:
            print("Error: missing _spi bus value")
            printHelp(1)
        del args[0]
        spi_bus = int(args[0])
        del args[0]
    elif args[0] == "-e" or args[0] == "--ce":
        if len(args) < 2:
            print("Error: missing CE pin value")
            printHelp(1)
        del args[0]
        ce_gpio_pin = int(args[0])
        del args[0]
    elif args[0] == "-t" or args[0] == "--timeout":
        if len(args) < 2:
            print("Error: missing timeout value")
            printHelp(1)
        del args[0]
        timeout = float(args[0])
        del args[0]
    elif args[0] == "-v" or args[0] == "--verbose":
        del args[0]
        verbose += 1
    elif args[0] == "-vv":
        del args[0]
        verbose += 2
    elif args[0] == "-vvv":
        del args[0]
        verbose += 3

if verbose > 0:
    print("_spi bus is " + str(spi_bus))
    print("CE pin is " + str(ce_gpio_pin))
    print("Address is " + str(address))
    print("Channel is " + str(channel))
    print("Timeout is " + str(timeout) + "s")


def sendAndReceiveData(data):
    if verbose > 1:
        print("> " + str(data))

    data.insert(0, len(data))
    if verbose > 2:
        print("T " + str(data))

    res = nRF2401.sendAndReceive(data, timeout)

    if verbose > 2:
        print("R " + str(res))

    length = res[0]
    del res[0]
    for i in range(len(res) - 1, length - 1, -1):
        del res[i]

    if verbose > 1:
        print("< " + str(res))

    return res


def valid(buf):
    return len(buf) > 1 and buf[0] == 0x14 and buf[len(buf) - 1] == 0x10


def sync():
    if verbose > 0:
        print("Syncing...")
    sendAndReceiveData([0x30, 0x20])
    sendAndReceiveData([0x30, 0x20])
    response = sendAndReceiveData([0x30, 0x20])

    retries = 1
    while not valid(response) and retries < 10:
        retries -= 1
        if verbose > 1:
            print("Error trying to sync. " + str(retries) + " of 10 retries...")
        response = sendAndReceiveData([0x30, 0x20])

    if verbose > 1:
        print("Syncing...done.")


def signature():
    sync()
    response = sendAndReceiveData([0x75, 0x20])
    if len(response) != 5 or not valid(response):
        print("Error: Wrong response to signature 1 request; " + str(response))
        sys.exit(1)

    s1 = response[1]
    s2 = response[2]
    s3 = response[3]

    print()
    print("Device's signature code is 0x" + hex(s1) + " 0x" + hex(s2) + " 0x" + hex(s3))


def version():
    sync()
    response = sendAndReceiveData([0x41, 0x80, 0x20])
    if len(response) != 3 or not valid(response):
        print("Error: Wrong response to board 0x80 request; " + str(response))
        sys.exit(1)
    hwVer = response[1]

    response = sendAndReceiveData([0x41, 0x81, 0x20])
    if len(response) != 3 or not valid(response):
        print("Error: Wrong response to board 0x81 request; " + str(response))
        sys.exit(1)
    majorVer = response[1]

    response = sendAndReceiveData([0x41, 0x82, 0x20])
    if len(response) != 3 or not valid(response):
        print("Error: Wrong response to board 0x82 request; " + str(response))
        sys.exit(1)
    minorVer = response[1]

    pageSize = readFlashPageSize()

    print("Device's version - hw: " + hex(hwVer) + ", major: " + hex(majorVer) + ", minor " + hex(minorVer))
    print("Device's flash page size " + str(pageSize))


def readFlashPageSize():
    response = sendAndReceiveData([0x41, 0xC7, 0x20])
    if len(response) != 4 or not valid(response):
        print("Error: Wrong response to board 0xC7 request; " + str(response))
        sys.exit(1)
    sizeL = response[1]
    sizeH = response[2]
    return sizeL + 256 * sizeH


def readId():
    sync()
    response = sendAndReceiveData([0x31, 0x20])
    if len(response) != 9 or not valid(response):
        print("Error: Wrong response to read id request; " + str(response))
        sys.exit(1)

    s = chr(response[1]) + chr(response[2]) + chr(response[3]) + chr(response[4]) + chr(response[5]) + chr(response[6]) + chr(response[7])

    print()
    print("Programmer Id: " + s)


def loadHexFile(pageSize, fileName, dump):
    pages = {}

    def getPage(pageAddr):
        if pageAddr in pages:
            pageRes = pages[pageAddr]
        else:
            pageRes = []
            for ii in range(0, pageSize):
                pageRes.append(0)
            pages[pageAddr] = pageRes
        return pageRes

    lineNo = 0
    with open(fileName, "r") as file:
        for line in file:
            lineNo += 1
            if not line.startswith(":"):
                print("Bad hex format. Line " + str(lineNo) + " does not start with ':'. Line='" + line + "'")
            cnt = int(line[1:3], 16)
            adr = int(line[3:7], 16)
            typ = int(line[7:9], 16)
            # chsum = int(line[len(line) - 2:], 16)
            if typ == 0:
                pageAdr = adr // pageSize
                page = getPage(pageAdr)
                a = adr - pageAdr * pageSize

                for i in range(0, cnt):
                    if a >= pageSize:
                        pageAdr += 1
                        a = 0
                        page = getPage(pageAdr)

                    d = int(line[9 + i * 2: 9 + i * 2 + 2], 16)
                    page[a] = d
                    a += 1

    if dump:
        row = 0
        i = 0
        for pageAddress in pages:
            localAddress = pageAddress * pageSize
            response = pages[pageAddress]
            length = len(response)

            print("0x{:04x}: ".format(localAddress), end="")
            while length > 0:
                print("0x{:02x} ".format(response[i]), end="")
                length -= 1
                row += 1
                if row == 8:
                    row = 0
                    print()
                    localAddress += 8
                    if length > 0:
                        print("0x{:04x}: ".format(localAddress), end="")
                i += 1
                if i == len(response):
                    response = sendAndReceiveData([])
                    i = 0
            if row != 0:
                print()
            print()

    return pages


def doVerify(pageSize, pages, eeprom):
    if verbose > 0:
        print("Verifying written data")

    error = False
    for pageAdr in pages.keys():
        if verbose > 0:
            print("Verifying page " + str(pageAdr + 1) + " of " + str(len(pages)), end="... ")
        localAddress = pageAdr * pageSize
        page = pages[pageAdr]

        data = [0x55, localAddress % 256, localAddress // 256, 0x20]
        response = sendAndReceiveData(data)
        if len(response) != 2 and not valid(response):
            print("Error: Wrong response to set address request; " + str(response))
            sys.exit(1)

        if eeprom:
            destination = ord('E')
        else:
            destination = ord('F')

        data = [0x74, pageSize // 256, pageSize % 256, destination, 0x20]
        response = sendAndReceiveData(data)
        if len(response) < 3 or response[0] != 0x14:
            print("Error: Wrong response to read data request; " + str(response))
            sys.exit(1)

        length = pageSize
        i = 1
        a = 0
        while length > 0:
            length -= 1
            if not error and response[i] != page[a]:
                print()
                print("Error: verify failed at byte " + str(localAddress + a) + ", local:" + hex(page[a]) + " != " + hex(response[i]))
                error = True
            i += 1
            a += 1
            if i == len(response):
                response = sendAndReceiveData([])
                i = 0
            if length == 0:
                if response[i] != 0x10:
                    print()
                    print("Error: Wrong response to read data request; didn't receive 0x10; " + str(response))

        if error:
            sys.exit(1)
        else:
            if verbose > 0:
                print("done.")

    if verbose > 0:
        print("Verifying written data done. All OK.")


def write():
    global args

    del args[0]

    dumpHex = False
    start = True
    verifyFlag = True
    eeprom = False
    fileName = None
    pageSize = 0

    while len(args) > 0:
        if args[0] == "-p" or args[0] == "--page":
            if len(args) < 2:
                print("Error: missing page size")
                printHelp(1)
            else:
                pageSize = int(args[1])
                del args[0]
                del args[0]
        elif args[0] == "-e" or args[0] == "--eeprom":
            eeprom = True
            del args[0]
        elif args[0] == "-f" or args[0] == "--flash":
            eeprom = False
            del args[0]
        elif args[0] == "-s" or args[0] == "--start":
            start = True
            del args[0]
        elif args[0] == "-n" or args[0] == "--dont-start":
            start = False
            del args[0]
        elif args[0] == "-t" or args[0] == "--verify":
            verifyFlag = True
            del args[0]
        elif args[0] == "-N" or args[0] == "--dont-verify":
            verifyFlag = False
            del args[0]
        elif args[0] == "-d" or args[0] == "--dump-hex":
            dumpHex = True
            del args[0]
        else:
            fileName = args[0]
            if len(args) > 1:
                print("Error: too menu args after " + str(args[0]))
                printHelp(1)
            del args[0]

    if fileName is None:
        print("Error: missing file name")
        printHelp(1)

    if not eeprom and pageSize == 0:
        pageSize = readFlashPageSize()
        if verbose > 0:
            print("Flash page size " + str(pageSize))

    if not os.path.exists(fileName):
        print("Error: file does not exist; '" + fileName + "'")
        sys.exit(1)

    pages = loadHexFile(pageSize, fileName, dumpHex)

    sync()

    for pageAdr in pages.keys():
        if verbose > 0:
            print("Sending page " + str(pageAdr + 1) + " of " + str(len(pages)), end="... ")
        localAddress = pageAdr * pageSize
        data = [0x55, localAddress % 256, localAddress // 256, 0x20]
        response = sendAndReceiveData(data)
        if len(response) != 2 and not valid(response):
            print("Error: Wrong response to set address request; " + str(response))
            sys.exit(1)

        page = pages[pageAdr]

        dest = ord('F')
        if eeprom:
            dest = ord('E')

        data = [0x64, pageSize // 256, pageSize % 256, dest]
        a = 0
        b = 0
        while a < pageSize:
            while a < pageSize and len(data) < DATA_SIZE:
                data.append(page[a])
                a += 1

            if a == pageSize and len(data) < DATA_SIZE:
                data.append(0x20)

            b = sendAndReceiveData(data)
            data = []

        if not valid(b):
            if verbose > 0:
                print("error.")
            print("Error: received invalid response; " + str(b))
            sys.exit(1)
        else:
            if verbose > 0:
                print("done.")

    if verbose > 0:
        print("Sent " + str(len(pages)) + " pages")

    if verifyFlag:
        doVerify(pageSize, pages, eeprom)

    if not eeprom and start:
        data = [0x51, 0x20]
        b = sendAndReceiveData(data)
        if not valid(b):
            print("Got wrong response for start/reset; " + str(b))
            sys.exit(1)
        if verbose > 0:
            print("Starting code.")


def verify():
    dumpHex = False
    eeprom = False
    fileName = None
    pageSize = 0

    del args[0]

    while len(args) > 0:
        if args[0] == "-p" or args[0] == "--page":
            if len(args) < 2:
                print("Error: missing page size")
                printHelp(1)
            else:
                pageSize = int(args[1])
                del args[0]
                del args[0]
        elif args[0] == "-e" or args[0] == "--eeprom":
            eeprom = True
            del args[0]
        elif args[0] == "-d" or args[0] == "--dump-hex":
            dumpHex = True
            del args[0]
        elif args[0] == "-f" or args[0] == "--flash":
            eeprom = False
            del args[0]
        else:
            fileName = args[0]
            if len(args) > 1:
                print("Error: too menu args after " + str(args[0]) + "; " + str(args[1]))
                printHelp(1)
            del args[0]

    if fileName is None:
        print("Error: missing file name")
        printHelp(1)

    if not eeprom and pageSize == 0:
        pageSize = readFlashPageSize()
        if verbose > 0:
            print("Flash page size " + str(pageSize))

    if not os.path.exists(fileName):
        print("Error: file does not exist; '" + fileName + "'")
        sys.exit(1)

    pages = loadHexFile(pageSize, fileName, dumpHex)

    sync()

    doVerify(pageSize, pages, eeprom)


def read():

    bytesPerRow = 8
    del args[0]

    eeprom = False
    addressRange = False
    endAddress = 0
    localAddress = 0

    while len(args) > 0:
        if args[0] == "-e" or args[0] == "--eeprom":
            eeprom = True
            del args[0]
        elif args[0] == "-f" or args[0] == "--flash":
            eeprom = False
            del args[0]
        elif args[0] == "-w" or args[0] == "--wide":
            bytesPerRow = 64
            del args[0]
        elif args[0] == "-l" or args[0] == "--long":
            bytesPerRow = 256
            del args[0]
        else:
            split = args[0].split(':')
            if len(split) != 2:
                print("Error: missing <start:end>")
                printHelp(1)
            localAddress = int(split[0], 0)
            endAddress = int(split[1], 0)
            addressRange = True
            if len(args) > 1:
                print("Error: too menu args after " + str(args[0]) + "; " + str(args[1]))
                printHelp(1)
            del args[0]

    if not addressRange:
        print("Error: missing <start:end>")
        printHelp(1)

    length = endAddress - localAddress
    if length <= 0:
        print("Error: invalid <start:end>")
        printHelp(1)

    sync()

    data = [0x55, localAddress % 256, localAddress // 256, 0x20]
    response = sendAndReceiveData(data)
    if len(response) != 2 and not valid(response):
        print("Error: Wrong response to set address request; " + str(response))
        sys.exit(1)

    if eeprom:
        destination = ord('E')
    else:
        destination = ord('F')

    data = [0x74, length // 256, length % 256, destination, 0x20]
    response = sendAndReceiveData(data)
    if len(response) < 3 or response[0] != 0x14:
        print("Error: Wrong response to read data request; " + str(response))
        sys.exit(1)

    row = 0
    i = 1
    print("0x{:04x}: ".format(localAddress), end="")
    while length > 0:
        if bytesPerRow > 16:
            print("{:02x}".format(response[i]), end="")
        else:
            print("0x{:02x} ".format(response[i]), end="")
        length -= 1
        row += 1
        if row == bytesPerRow:
            row = 0
            print()
            localAddress += bytesPerRow
            if length > 0:
                print("0x{:04x}: ".format(localAddress), end="")
        i += 1
        if i == len(response):
            response = sendAndReceiveData([])
            i = 0
        if length == 0:
            if response[i] != 0x10:
                print("Error: Wrong response to read data request; didn't receive 0x10; " + str(response))
    if row != 0:
        print()
    print()


def reset():
    sync()

    data = [0x51, 0x20]
    b = sendAndReceiveData(data)
    if not valid(b):
        print("Got wrong response for start/reset; " + str(b))
        sys.exit(1)


try:
    if verbose > 0:
        print("Initialising ..")

    init()

    if verbose > 0:
        print("Initialised ")

    if len(args) < 1:
        print("Error: missing command.")
        printHelp(1)
    elif args[0] == "version":
        version()
    elif args[0] == "signature":
        signature()
    elif args[0] == "readid":
        readId()
    elif args[0] == "read":
        read()
    elif args[0] == "verify":
        verify()
    elif args[0] == "write":
        write()
    elif args[0] == "reset":
        reset()


except KeyboardInterrupt:
    nRF2401.powerDown()
    nRF2401.close()
