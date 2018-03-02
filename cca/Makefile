APPLICATION = cca
RIOTBASE = /root/RIOT-OS
BOARD = samr21-xpro

# This is to set the proper programmer for board
PROGRAMMER ?= jlink

# System functions
USEMODULE += random
USEMODULE += xtimer
USEMODULE += rtt_stdio
CFLAGS    += -DRTT_STDIO_DISABLE_STDIN
CFLAGS    += -DTHREAD_STACKSIZE_DEFAULT=2048

# CPU clock speed
CFLAGS    += -DCLOCK_USE_OSCULP32_DFLL=1
CFLAGS    += -DCLOCK_USE_ADAPTIVE=0
CFLAGS    += -DDMAC_ENABLE=0

# Radio
USEMODULE += at86rf2xx
CFLAGS    += -DIEEE802154_DEFAULT_CHANNEL=26
CFLAGS    += -DAUTO_CSMA_EN=1 
CFLAGS    += -DCONTINUOUS_CCA=1

# Network
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif

# IPv6 extension headers
USEMODULE += gnrc_sixlowpan
# UDP
USEMODULE += gnrc_udp

# Duty-cycling 
USEMODULE += gnrc_lasmac
CFLAGS    += -DLEAF_NODE=0                       # Default is 1  
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6
USEMODULE += gnrc_udp
CFLAGS    += -DSOCK_HAS_IPV6
# Add also the shell, some shell commands
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6


# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
# CFLAGS += -DDEVELHELP

QUIET ?= 1

FEATURES_REQUIRED += periph_timer

include $(RIOTBASE)/Makefile.include
