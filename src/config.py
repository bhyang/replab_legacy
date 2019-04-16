import numpy as np
from geometry_msgs.msg import Quaternion

# MODELS
PINTO2016_PRETRAINED_WEIGHTS = '/root/widowx_arm/src/replab/src/pinto-model.th'
FULLIMAGE_PRETRAINED_WEIGHTS = '/home/dianchen/grasping/result/full-image/model.th'
METHODS = ('datacollection', 'datacollection-noiseless',
           'principal-axis', 'pinto2016', 'fullimage', 'custom', 'combined')

# DATA COLLECTION
DATAPATH = ''  # path for saving data collection samples
XY_NOISE = .02


# CONTROL
PRELIFT_HEIGHT = .39
Z_OFFSET = 0.04
Z_MIN = .44
CONTROL_NOISE_COEFFICIENT = 1.15


# CAMERA
CALIBRATION_MATRIX = np.array([[-1.00503107,  0.00415679,  0.05496505, -0.05620065],
                               [0.04065896, -0.95266129,  0.38997774, -0.2189197],
                               [0.02370495,  0.46873043,  0.95348986, -0.0564024],
                               [0.,  0.,  0.,  1.]])

MAX_DEPTH = 700.0


# BLOB DETECTION
DBSCAN_EPS = .03
DBSCAN_MIN_SAMPLES = 8
PC_BOUNDS = [(-.19,  .20),
             (.19,  .20),
             (.19, -.24),
             (-.19, -.23)]
HEIGHT_BOUNDS = (.39, .49)
PC_BASE_PATH = 'pc_base.npy'


# EMAIL SETTINGS
TOADDR = ''
FROMADDR = ''
FROMADDR_PASSWORD = ''


# ARENA BOUNDARIES
# WARNING: modifying these values may lead to unsafe/volatile arm behavior
END_EFFECTOR_BOUNDS = [(.15,  .15),
                       (.14, -.16),
                       (-.17, -.16),
                       (-.16,  .15)]

# CONTROLLER CONSTANTS AND PREPLANNED ROUTINES
DROPPING_VALUES = [0.0, -1.418932228794218,
                   1.3575729972787924, -0.8512816004258118, 0.0]
NEUTRAL_VALUES = [0.015339807878856412, -1.4839419194602816,
                  1.4971652489763858, -0.008369006790373335, -0.08692557798018634]
EMPTY_VALUES = [0.015339807878856412, -1.2931458041875956,
                1.5109710760673565, .55, -0.07158577010132992]
RESET_VALUES = [0.015339807878856412, -1.2931458041875956,
                1.0109710760673565, -1.3537670644267164, -0.07158577010132992]

PREDISCARD_VALUES = [1.564660403643354, -1.4081943632790186,
                     1.3437671701878218, -0.006135923151542565, -0.02556634646476069]
DISCARD_VALUES = [1.5539225381281545, 0.43104860139586515, -
                  0.3506991028509451, -1.3062720155208536, -0.07669903939428206]

SWEEPING_VALUES = [0.0, -0.4279806398200939,
                   0.46479617872934925, -1.6444274046134073, -0.015339807878856412]

DOWN_ORIENTATION = Quaternion(
    x=-0.00340167509175,
    y=-0.0109908935017,
    z=-0.246434325744,
    w=0.969091198991
)

GRIPPER_DROP = [0.031, 0.031]
GRIPPER_OPEN = [0.027, 0.027]
GRIPPER_CLOSED = [.003, .003]

TL_CORNER = [[0.8268156446703606, -0.14419419406125028, -0.09817477042468103, -0.8084078752157329, -0.0051132692929521375],
             [0.8268156446703606, -0.14419419406125028, -0.09817477042468103, -1.5084078752157329, -0.0051132692929521375]]
L_SWEEP = [[0.8268156446703606, -0.14419419406125028, -0.09817477042468103, -1.3084078752157329, -0.0051132692929521375],
           [-0.8268156446703606, -0.14419419406125028, -0.09817477042468103, -1.3084078752157329, -0.0051132692929521375]]
BL_CORNER = [[-0.8268156446703606, -0.14419419406125028, -0.09817477042468103, -0.6084078752157329, -0.0051132692929521375],
             [-0.8268156446703606, -0.14419419406125028, -0.09817477042468103, -1.5084078752157329, -0.0051132692929521375]]
BR_CORNER = [[-2.2181362192826373, -0.04283496254582462, -0.0841359443037636, -0.8031845668518499, -0.02045307717180855],
             [-2.2181362192826373, -0.08283496254582462, -0.1641359443037636, -1.5531845668518498, -0.02045307717180855]]
TR_CORNER = [[2.376136240434858, -0.2715145994557585, -0.007669903939428206, -0.624330180669456, -0.0409061543436171],
             [2.376136240434858, -0.2715145994557585, -0.007669903939428206, -1.524330180669456, -0.0409061543436171]]

UPRIGHT_DROP = [0., -0.34821363885004053, 0.7470486437003072, 1.2041749184902284, -0.0051132692929521375, 0.003]
UPRIGHT_NEUTRAL = [0., -1.1642914180052018, 1.239456476611598, -0.10737865515199488, 0.0051132692929521375, 0.003]
UPRIGHT_RESET = [0., -0.4, 0.7915340865489908, -0.3942330624866098, -0.010226538585904275, 0.003]
