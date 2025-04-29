            if i == 0:
                t_motor_start = time.time()  # Start time of the loop
            t_motor_now = time.time() - t_motor_start  # Elapsed time
            omega = 0.25
            i += 1

            if t_motor_now >= 12:
                omega = 0
            elif t_motor_now >= 9:
                omega = 1.00
            elif t_motor_now >= 6:
                omega = 0.75
            elif t_motor_now >= 3:
                omega = 0.5

            Tx = signals["left_thumbstick_x"] * TX_MAX
            Ty = signals["left_thumbstick_y"] * TY_MAX
            #Tz = signals["right_thumbstick_x"] * TZ_MAX
            #Tz = signals["trigger_R2"] * TZ_MAX
            Tz = omega
