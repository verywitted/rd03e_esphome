public:
  enum class ParamRef : uint16_t {
    DISTANCE_CALIBRATION = 0x0000,
    MAX_MOVEMENT_DISTANCE = 0x0000,
    MIN_MOVEMENT_DISTANCE = 0x0001,
    MAX_MICRO_DISTANCE = 0x0002,
    MIN_MICRO_DISTANCE = 0x0003,
    UNMANNED_DURATION = 0x0004,
    PROXIMAL_MOTION_NOISE = 0x0000,
    DISTAL_MOTION_NOISE = 0x0001,
    PROXIMAL_MICRO_NOISE = 0x0002,
    DISTAL_MICRO_NOISE = 0x0003,
    MOTION_CLUTTER_SUPPRESS = 0x0000,
    MICRO_CLUTTER_SUPPRESS = 0x0001,
    MOTION_FRAME_WINDOW = 0x0000,
    MICRO_FRAME_WINDOW = 0x0001,
    ALPHA_BETA_FILTER_1 = 0x0000,
    ALPHA_BETA_FILTER_2 = 0x0001,
    ALPHA_BETA_FILTER_3 = 0x0002,
    ALPHA_BETA_FILTER_4 = 0x0003
  };
  
  // Configuration struct
    struct RD03EConfig {
      // Flag to track which parameters have been changed
      struct {
        bool distanceCalibration: 1;
        bool distanceSettings: 1;
        bool noiseParams: 1;
        bool clutterParams: 1;
        bool frameWindowParams: 1;
        bool alphaParams: 1;
      } modified = {0};
      
      // Distance calibration parameter (0x0072)
      int32_t distanceCalibration = 0;
      
      // Distance settings (0x0067)
      struct {
        uint32_t maxMovement = 717;      // 30-717 cm
        uint32_t minMovement = 30;       // Minimum value
        uint32_t maxMicroMotion = 425;   // 30-425 cm
        uint32_t minMicroMotion = 30;    // Minimum value
        uint32_t unmannedDuration = 20;  // 0-65535 (in units of 50ms)
      } distanceSettings;
      
      // Noise parameters (0x0068)
      struct {
        float proximalMotion = 40.0f;
        float distalMotion = 6.0f;
        float proximalMicro = 40.0f;
        float distalMicro = 9.0f;
      } noiseParams;
      
      // Clutter suppression (0x0069)
      struct {
        uint32_t motionBranch = 2;  // 0-255
        uint32_t microBranch = 8;   // 0-255
      } clutterParams;
      
      // Frame window length (0x0070)
      struct {
        uint32_t motionBranch = 5;  // 0-255
        uint32_t microBranch = 10;  // 0-255
      } frameWindowParams;
      
      // Alpha-beta filter coefficients (0x0071)
      struct {
        float coef1 = 0.5f;
        float coef2 = 0.5f;
        float coef3 = 0.85f;
        float coef4 = 0.15f;
      } alphaParams;
    };