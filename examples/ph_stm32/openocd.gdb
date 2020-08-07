target remote :3333
set print asm-demangle on
set print pretty on

load

monitor arm semihosting enable

# Detect unhandled exceptions, hard faults and panics
break DefaultHandler
break main
continue
continue