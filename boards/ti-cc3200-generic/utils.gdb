
define reset
  monitor reset init
  monitor reset halt
  set $sp = __interrupt_vector[0]
  set $pc = __interrupt_vector[1]
  continue
end
