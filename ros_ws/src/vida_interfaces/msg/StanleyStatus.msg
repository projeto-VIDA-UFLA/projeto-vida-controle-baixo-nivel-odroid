# Diagnostico do seguidor de caminho (vida_navigation/stanley), publicado a
# cada odom. O mode_manager usa state e cross_track_m para os abortos.
builtin_interfaces/Time timestamp

uint8 STATE_IDLE=0          # desabilitado (modo manual ou preparando)
uint8 STATE_TRACKING=1      # seguindo o caminho
uint8 STATE_STOPPING=2      # perto do fim: desacelerando
uint8 STATE_FINISHED=3      # chegou ao fim e parou
uint8 STATE_NO_ODOM=4       # odom do EKF atrasada ou sem origem: sem comando
uint8 state

string path_name
uint32 path_index           # segmento projetado (eixo dianteiro)
float32 path_s_m            # posicao ao longo do caminho
float32 progress_m          # percorrido desde a habilitacao
float32 remaining_m         # ate o fim da volta / do caminho

float32 cross_track_m       # eixo dianteiro, positivo com o veiculo a ESQUERDA do caminho
float32 heading_error_rad   # rumo do caminho - rumo do veiculo
float32 path_curvature_1_m

float32 steering_heading_deg     # parcela do erro de rumo
float32 steering_cross_track_deg # parcela do erro lateral
float32 steering_feedforward_deg # parcela da curvatura
float32 steering_command_deg     # apos saturacao e limite de taxa
bool steering_saturated
float32 speed_command_m_s
