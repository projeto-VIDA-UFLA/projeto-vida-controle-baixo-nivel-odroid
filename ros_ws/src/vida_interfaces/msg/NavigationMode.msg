# Estado do mode_manager (vida_navigation), publicado a 10 Hz e a cada troca.
builtin_interfaces/Time timestamp

uint8 MODE_MANUAL=0        # gamepad comanda o veiculo
uint8 MODE_PREPARING=1     # parado, alinhando as rodas em 0 grau
uint8 MODE_CALIBRATING=2   # parado, EKF reiniciado: origem (3 fixes) e bias
uint8 MODE_MANEUVER=3      # reta a 0,5 m/s ate o EKF ter o heading pelo GPS
uint8 MODE_AUTOMATIC=4     # Stanley seguindo o caminho
uint8 MODE_STOPPING=5      # parando (fim do caminho, saida pedida ou aborto)
uint8 mode

bool aborted               # a ultima saida do automatico foi um aborto
uint32 refused_count       # pedidos de automatico recusados (o gamepad vibra a cada novo)
string reason              # motivo da ultima troca de modo (texto para o operador)
string path_name
float32 progress_m         # Stanley: percorrido / restante
float32 remaining_m
