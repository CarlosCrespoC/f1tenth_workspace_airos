def _update_pose(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        
        # Comprueba si los datos de AMCL son recientes y válidos
        amcl_ok = (self.use_amcl and self.have_amcl and (now - self.last_amcl_t) <= self.amcl_timeout)
        
        if amcl_ok:
            # Si AMCL está bien, actualiza la pose del vehículo con sus datos
            self.x = self.x_amcl
            self.y = self.y_amcl
            self.yaw = self.yaw_amcl
            self.using_amcl = True
            return True # Retorna True para indicar que la pose es válida
        else:
            # Si no hay datos de AMCL válidos, no se actualiza la pose y 
            # se retorna False. Esto hará que el vehículo desacelere
            # de forma controlada en la función _on_timer.
            self.using_amcl = False
            return False
