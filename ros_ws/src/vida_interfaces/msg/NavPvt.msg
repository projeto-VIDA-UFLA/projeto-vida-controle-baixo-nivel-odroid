# UBX-NAV-PVT completo (u-blox ZED-F9R, protocolo 33.30), campos e unidades
# nativos do receptor. Publicado a cada solucao, com ou sem fix.

# Instante de recepcao no host (relogio ROS).
builtin_interfaces/Time timestamp
string frame_id

uint32 i_tow        # ms, tempo da semana GPS da epoca de navegacao

# Data/hora UTC
uint16 year         # ano (UTC)
uint8 month         # mes, 1..12
uint8 day           # dia do mes, 1..31
uint8 hour          # hora, 0..23
uint8 min           # minuto, 0..59
uint8 sec           # segundo, 0..60

uint8 valid         # bits de validade da data/hora
uint8 VALID_DATE=1
uint8 VALID_TIME=2
uint8 VALID_FULLY_RESOLVED=4
uint8 VALID_MAG=8

uint32 t_acc        # ns, estimativa de acuracia do tempo UTC
int32 nano          # ns, fracao de segundo (-1e9..1e9)

uint8 fix_type
uint8 FIX_TYPE_NO_FIX=0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY=1
uint8 FIX_TYPE_2D=2
uint8 FIX_TYPE_3D=3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING=4
uint8 FIX_TYPE_TIME_ONLY=5

uint8 flags
uint8 FLAGS_GNSS_FIX_OK=1
uint8 FLAGS_DIFF_SOLN=2
uint8 FLAGS_PSM_MASK=28
uint8 FLAGS_HEAD_VEH_VALID=32
uint8 FLAGS_CARRIER_SOLN_MASK=192
uint8 FLAGS_CARRIER_SOLN_NONE=0
uint8 FLAGS_CARRIER_SOLN_FLOAT=64
uint8 FLAGS_CARRIER_SOLN_FIXED=128

uint8 flags2
uint8 FLAGS2_CONFIRMED_AVAI=32
uint8 FLAGS2_CONFIRMED_DATE=64
uint8 FLAGS2_CONFIRMED_TIME=128

uint8 num_sv        # satelites usados na solucao

int32 lon           # deg * 1e-7
int32 lat           # deg * 1e-7
int32 height        # mm, altura sobre o elipsoide
int32 h_msl         # mm, altura sobre o nivel medio do mar
uint32 h_acc        # mm, acuracia horizontal
uint32 v_acc        # mm, acuracia vertical

int32 vel_n         # mm/s, velocidade NED norte
int32 vel_e         # mm/s, velocidade NED leste
int32 vel_d         # mm/s, velocidade NED baixo
int32 g_speed       # mm/s, velocidade no solo (2D)
int32 head_mot      # deg * 1e-5, rumo do movimento (2D)
uint32 s_acc        # mm/s, acuracia da velocidade
uint32 head_acc     # deg * 1e-5, acuracia do rumo (movimento e veiculo)

uint16 p_dop        # * 0.01, DOP de posicao

uint16 flags3
uint16 FLAGS3_INVALID_LLH=1
uint16 FLAGS3_LAST_CORRECTION_AGE_MASK=30

uint8[4] reserved0

int32 head_veh      # deg * 1e-5, rumo do veiculo (valido se FLAGS_HEAD_VEH_VALID)
int16 mag_dec       # deg * 1e-2, declinacao magnetica
uint16 mag_acc      # deg * 1e-2, acuracia da declinacao magnetica
