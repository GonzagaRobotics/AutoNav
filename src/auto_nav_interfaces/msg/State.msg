# 0: DISABLED - AutoNav is disabled and won't do anything
# 1: READY - AutoNav is ready to receive a target
# 2: PLANNING - AutoNav is making a plan to reach the target
# 3: WAITING - AutoNav has a plan and is waiting for an instruction
# 4: TRAVELING - AutoNav is traveling towards the target
# 5: TERMINAL_SEARCHING - AutoNav is searching for the target
# 6: TERMINAL_MOVING - AutoNav found the target and is moving towards it
# 7: PAUSED - AutoNav is paused
# 8: SUCCESS - AutoNav reached the target
# 9: FAILURE - AutoNav is unable to reach the target
uint8 state