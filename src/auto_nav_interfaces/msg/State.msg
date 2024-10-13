# 0: DISABLED - AutoNav is disabled and won't do anything
# 1: READY - AutoNav is ready to receive a target
# 2: PLANNING - AutoNav is making a plan to reach the target
# 3: TRAVELING - AutoNav is traveling towards the target
# 4: TERMINAL_SEARCHING - AutoNav is searching for the target
# 5: TERMINAL_MOVING - AutoNav found the target and is moving towards it
# 6: SUCCESS - AutoNav reached the target
# 7: FAILURE - AutoNav is unable to reach the target
uint8 state