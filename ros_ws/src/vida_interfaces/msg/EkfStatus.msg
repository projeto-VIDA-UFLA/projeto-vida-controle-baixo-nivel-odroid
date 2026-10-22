# Estado do robot_state_ekf, publicado continuamente (inclusive antes de inicializar).
builtin_interfaces/Time timestamp

uint8 PHASE_WAITING_ORIGIN=0   # aguardando os fixes que definem a origem local
uint8 PHASE_WAITING_HEADING=1  # origem fixada, aguardando deslocamento para o heading
uint8 PHASE_RUNNING=2          # filtro rodando, odom publicado
uint8 phase

uint8 HEADING_NONE=0
uint8 HEADING_GPS=1            # rumo do GPS (deslocamento entre fixes)
uint8 HEADING_PARAMETER=2      # initial_heading_deg (sem deslocamento suficiente)
uint8 heading_source

# Origem do frame local (odom). origin_id muda a cada nova origem (boot ou reset):
# coordenadas x, y de origens diferentes nao sao comparaveis.
bool origin_valid
float64 origin_latitude_deg
float64 origin_longitude_deg
uint32 origin_id

bool stationary                # veiculo parado pelos encoders
float64 gyro_bias_rad_s        # bias estimado do giroscopio Z
uint32 bias_calibration_samples  # amostras paradas usadas no bias inicial
uint8 gps_rtk_status           # ultimo GnssStatus.rtk_status (RTK_NONE sem status)
