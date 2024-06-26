

set(PRJ EMAC_lwiperf)

set(EMAC_DRV
    ./drv_emac/m460_emac.c
    ./drv_emac/m460_mii.c
    ./drv_emac/synopGMAC_Dev.c
    ./drv_emac/synopGMAC_network_interface.c
    ./drv_emac/synopGMAC_plat.c
)

set(LWIP_SRC
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/apps/lwiperf/lwiperf.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/def.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/dns.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/inet_chksum.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/init.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ip.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/mem.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/memp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/netif.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/pbuf.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/raw.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/stats.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/sys_lwip.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/tcp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/tcp_in.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/tcp_out.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/timeouts.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/udp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/autoip.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/dhcp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/etharp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/icmp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/igmp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/ip4.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/ip4_addr.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/core/ipv4/ip4_frag.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/netif/ethernet.c
)


add_executable(${PRJ} main.c queue.c sys_arch.c ${EMAC_DRV} ${LWIP_SRC})
target_link_libraries(${PRJ} system_startup standard_driver)
target_include_directories(${PRJ}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/drv_emac
        ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/lwIP/src/include

)

target_link_options(${PRJ}
   PUBLIC
    --cpu=Cortex-M4.fp.sp
    --library_type=microlib
    --ro-base 0x00000000
    --rw-base 0x20000000
    --entry Reset_Handler
    --first __Vectors
    --strict
    --map
    --datacompressor=off
    --info=inline
    --entry Reset_Handler
    --summary_stderr
    --info totals
    --map
    --load_addr_map_info
    --xref
    --callgraph
    --symbols
)

add_custom_command(TARGET ${PRJ} POST_BUILD
    COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
    VERBATIM
)

