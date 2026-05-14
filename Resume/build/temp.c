/*PRQA S 2995,3408,3332,0292,2741,3108,1265,4391,3218,4558,4116,4404,4558,4394,2983,4454,1330,2991,3415,2982,0315 ++ */ /* No issue as variables change beyond scope of function */
/*PRQA S 1751,1802,4461,2214,2013,3224,3218,2004 ++ */ /* Required for functionality */
#include "US_SnrData_Typedefs_Global.h"
#include "US_ME_Typedefs_Global.h"
#include "US_Platform_Global.h"
#include "US_PlatformCfg_Global.h"
#include "US_ComInputBuffer_Global.h"
#include "US_ObjDet_Global.h"
#include "US_SysMgr_Global.h"
#include "US_Odometry_Global.h"
#include "US_ObjDetData.h"
#include "US_ObjDetTriang.h"
#include "US_ObjDetUtility.h"
#include "US_ObjDetMain.h"
#include "US_ObjDetPointMap.h"
#include "US_ObjDetHeight.h"
#include "US_SnrCfg_Global.h"
#include <limits.h>

#if (US_D_USE_FAPA_API == ME_TRUE)
#include "SignalDef.h"
#endif

#define US_D_OD_SIMPLE_AVERAGE  (0.5f)

/*
 *   **** CONFIDENCE ADJUSTMENT RATES *****
 */
#ifdef US_D_USE_SLOW_CONF_APP_3
    #define US_D_OD_DEFAULT_CONF      (29u)  // default confidence if getting a real value fails
    #define US_D_OD_INITAL_CONF_CLOSE (10u) // Closest points get priority
    #define US_D_OD_INITAL_CONF_STD   (1u) // Starting confidence for echo index zero
    #define US_D_OD_INITAL_CONF_DIR_ONLY    (1u)

    #define US_D_OD_MAX_CONF_FOR_FAR_PNTS   (30u)
    #define US_D_OD_MAX_CONF_FOR_FAR_PNTS__SLOWMODE (30u)
    #define US_D_OD_MIN_DIST_OF_FAR_PNT     (200u)

    #define US_D_OD_PNT_CONF_BOOST               (1u)   // Additional measurement boost
    #define US_D_OD_PNT_CONF_BOOST__SLOWMODE        (15u)
    #define US_D_OD_SIG_STRENGTH_SHFT            (3u)   // Scaling down of Signal Strength for confidence% (SignalStength >> US_D_OD_SIG_STRENGTH_SHFT)
    #define US_D_OD_SIG_STRENGTH_SHFT__SLOWMODE     (16u) //basically turned off
    #define US_D_OD_INITIAL_SIG_STR_SHFT         (2u)    //only for brand new points
    #define US_D_OD_INITIAL_SIG_STR_SHFT__SLOWMODE  (16u) //basically turned off
    #define US_D_OD_INIT_NOMEAS_DEC_RATE_EXP     (1u)   // Missed measurement exponent initial rate, geometric decrement rate,
                                                        // 2^US_D_OD_STD_DL01_23AE_PACANfix_HystOff_SecFix_Dec1EC_RATE, ++ every miss
    #define US_D_OD_NOMEAS_DEC_RATE_EXP          (2u)   /* for existing points */
#else
    #define US_D_OD_DEFAULT_CONF      (29u)  // default confidence if getting a real value fails
    #define US_D_OD_INITAL_CONF_CLOSE (40u) // Closest points get priority
    #define US_D_OD_INITAL_CONF_STD   (30u) // Starting confidence for echo index zero   ***** CHANGED from 1 *****
    #define US_D_OD_INITAL_CONF_DIR_ONLY    (10u) 	//  ***** CHANGED from 1 *****

    #define US_D_OD_MAX_CONF_FOR_FAR_PNTS   (49u)  		//***** CHANGED from 30 *****
    #define US_D_OD_MAX_CONF_FOR_FAR_PNTS__SLOWMODE (20u)   // ***** CHANGED from 15 *****
    #define US_D_OD_MIN_DIST_OF_FAR_PNT     (300u)			// ***** CHANGED from 200 *****

    #define US_D_OD_PNT_CONF_BOOST               (20u)   // Additional measurement boost  ***** CHANGED from 1 *****

    #define US_D_OD_PNT_CONF_BOOST__SLOWMODE            (6u) // ***** CHANGED from 15 *****
    #define US_D_OF_PNT_CONF_FAPA_SLOWMODE_XTRA_BOOST   (4u)
    #define US_D_OD_PNT_CONF_ADDER_SLOWMODE             (6u)
    #define US_D_OD_SIG_STRENGTH_SHFT                   (3u)   // Scaling down of Signal Strength for confidence% (SignalStength >> US_D_OD_SIG_STRENGTH_SHFT)
    #define US_D_OD_SIG_STRENGTH_SHFT__SLOWMODE         (16u) //basically turned off
    #define US_D_OD_INITIAL_SIG_STR_SHFT                (3u)    //only for brand new points  ***** CHANGED from 2 *****

    #define US_D_OD_INITIAL_SIG_STR_SHFT__SLOWMODE  (16u) //basically turned off
    #define US_D_OD_INIT_NOMEAS_DEC_RATE_EXP     (1u)   // Missed measurement exponent initial rate, geometric decrement rate,  ***** CHANGED from 2 *****
    #define US_D_OD_INIT_FAKENFD_NOMEAS_DEC_RATE_EXP    (6u)   // Missed measurement exponent initial rate, geometric decrement rate, for fake NFD
                                                        //2^US_D_OD_STD_DL01_23AE_PACANfix_HystOff_SecFix_Dec1EC_RATE, ++ every miss

	#define US_D_OD_VERY_SLOW_DEC_FOR_CLOSE_PNTS        (ME_FALSE)
#endif
/*
 *   **** END CONFIDENCE ADJUSTMENT RATES *****
 */
#define US_D_OD_MAYBE_CONF_CAP_NOT_DI  (30u)
#define US_D_OD_MAYBE_CONF_CAP_DI      (35u)
#define US_D_OD_CONF_CAP_LOCKOUT_RANGE (150u)

/* Lone matching values */
#define US_D_OD_LONE_POINT_MATCH_DIST_LIM   (1000u)      // Max distance for which to allow point matching of lone directs, with existing points.

/* Tube check */
#define US_D_OD_TUBE_CHK_FLANK_MIN_X_OFFSET     (-100.0f)  // [cm]
#define US_D_OD_TUBE_CHK_FLANK_MAX_X_OFFSET     (250.0f) // [cm]
#define US_D_OD_TUBE_CHK_TUBE_MIN_X_OFFSET      (100.0f) // [cm]
#define US_D_OD_TUBE_CHK_MAX_X_OFFSET           (400.0f) // [cm]
#define US_D_OD_TUBE_CHK_FAR_Y_OFFSET           (170.0f) // [cm]
#define US_D_OD_TUBE_CHK_CONF_CAP               (40u)    // [%]

#define US_D_OD_VEH_WIDTH_OFFSET                (30.0f)  // [cm]

#define US_D_OD_FLANK_ACTIVE_DIST   (200.0f) // [cm]
#define US_D_OD_FLANK_INHIBIT_DIST  (450.0f) // [cm]

#define US_D_OF_FLANK_MIN_CONF_TRIANG           (30u)
#define US_D_OF_FLANK_MIN_CONF_NON_TRIANG       (60u)
#define US_D_OF_FLANK_MIN_CONF_DIST_THRESH      (250u)

#define US_OD_CONF_CAP_FOR_GROUP  (40u)

/*
 *   **** END CONFIDENCE ADJUSTMENT RATES *****
 */

/*
 * LIMITS
 */
#ifdef US_D_USE_SLOW_CONF_APP_3
    #define US_D_OD_PNT_INITIAL_MAX_CONF    (35u)
#else
    #define US_D_OD_PNT_INITIAL_MAX_CONF    (60u)
#endif
#define US_D_OD_PNT_INITIAL_MAX_CONF__SLOWMODE  (20u)

#if 0
   #define US_D_OD_MIN_LONE_DIRECT_DIST    (200u)  //not used anymore
#endif

/*
 * ************** ALGORITHM SELECTION ON/OFF ************************
 */
#define US_D_OD_USE_LONE_DIR_PNT_MATCHING   (ME_TRUE)   // Match untriangulated directs, including NFD measurements, with existing points.
#if (US_D_OD_USE_LONE_DIR_PNT_MATCHING == ME_TRUE)
    #define US_D_OD_USE_LDPM_ONLY_REAR          (ME_FALSE)
    #define US_D_OD_USE_LDPM_DISABLE_IN_NOISY   (ME_FALSE)
#endif
#define US_D_OD_CAP_PNT_CONF_WHEN_FLANKED       (ME_TRUE)
#define US_D_OD_USE_TEMPORAL_FOR_SIDE_SNRS      (ME_FALSE)
#define US_D_OD_USE_RADIAL_MATCHING_FOR_TD      (ME_TRUE)
#define US_D_OD_USE_DI_MULTI_SNR_CCAP_RELEASE   (ME_FALSE)

#define US_D_OD_DEL_LONES_WHEN_NO_INNER_MEAS    (ME_TRUE)
#if (US_D_OD_DEL_LONES_WHEN_NO_INNER_MEAS == ME_TRUE)
    #define US_D_OD_WEAKEN_LONES_AT_DIST           (ME_TRUE)
    #define US_D_OD_WEAKEN_DIST_FOR_LONES          (200u)  // [cm]
#endif

/* Dynamic Target garbage collection. aka Dead Point Sweeper */
#define US_D_OD_DYN_TARG_DEAD_PNT_SWEEPER     (ME_TRUE)

#define US_D_OD_USE_ODOMETRY_FLAG_FUNC  (ME_TRUE)

#define US_D_OD_PATCH_PARALLEL_ECHOES_ONLY  (ME_TRUE)

#define US_D_OD_USE_SQUARE_FOV_CHECK (ME_FALSE)

#define US_D_OD_USE_FAST_SORT       (ME_FALSE)


#define US_D_TD_BENCH_TEST_MODE (ME_FALSE)  // Must be false for vehicle test!!!
#if (US_D_TD_BENCH_TEST_MODE == ME_FALSE)
    #define US_D_TD_UNIT_TESTING_MODE (0)  // 0 = No Test
    #define US_D_TD_ODO_UNIT_TESTING_MODE (0)
    #define US_D_TD_CACHE_UNIT_TESTING_MODE (ME_FALSE)
#else
    #define US_D_TD_UNIT_TESTING_MODE (1)
    #define US_D_TD_ODO_UNIT_TESTING_MODE (1)
    #define US_D_TD_CACHE_UNIT_TESTING_MODE (ME_TRUE)
#endif

#define US_D_OD_USE_INSTA_KILL_HIGH_CRUISE  (ME_FALSE)
#if (US_D_OD_USE_INSTA_KILL_HIGH_CRUISE == ME_TRUE)
    #define US_D_OD_INSTA_KILL_CRUISE_VAL   (16u)
#endif

/* Main define in header */
#if (US_D_OD_USE_CR_DIST_KILL == ME_TRUE)
    #define US_D_OD_USE_CRDK_ONLY_FRONT (ME_TRUE)
    #define US_D_OD_USE_CRDK_REAR_DIST_CM   80.0f /* used as positive and negative */
    #define US_D_OD_USE_CRDK_FRNT_DIST_CM   80.0f
    #define US_D_OD_USE_CRDK_IGNORE_CLOSE   (ME_TRUE)
    #if (US_D_OD_USE_CRDK_IGNORE_CLOSE == ME_TRUE)
        #define US_D_OD_USE_CRDK_CLOSE_CM   (50.0f)
    #endif
#endif

/* Related to group allow levels*/
#define US_D_OD_USE_NOISY_SWITCH_KILL   (ME_TRUE)
#if (US_D_OD_USE_NOISY_SWITCH_KILL == ME_TRUE)
    #define US_D_OD_USE_NSK_ONLY_FRONT (ME_FALSE)
    #define US_D_OD_USE_NSK_CYCLES      (10u)
#endif
#define US_D_OD_EGO_DIR_CHANGE_CONF_REDUCE    (ME_TRUE)
#if (US_D_OD_EGO_DIR_CHANGE_CONF_REDUCE == ME_TRUE)
    #define US_D_OD_USE_GSK_CYCLES      (1u) /* clear once and resume activities */
#endif

/* Reducing conf boost and reducing conf when distance capped for anything that isn't dir-indir triang */
#define US_D_OD_SLOW_N_CAP_NON_DIR_INDIR    (ME_FALSE)
#define US_D_OD_DEBUG_VARS_ON (ME_FALSE)

/* Necromancy */
#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
    #define US_D_OD_KDP_ACCEPT_CONF     (25u) /* pt must reach this confidence to be kept later */
    #define US_D_OD_KDP_DEAD_PT_CONF    (46u) /* Artificial conf to keep the dead pt at */
    #define US_D_OD_KDP_REJECT_LONES    (ME_FALSE)
#if 0
    #define US_D_OD_KDP_BOOST_FRNT_SIDES     (ME_TRUE)
#endif
    #define US_D_OD_KDP_CHECK_SIDES_ONLY   (ME_TRUE)
#if 0
    #define US_D_OD_KDP_HACKY_REAR_SIDE_ADJ (0.17f) /* shifting rear side lones in rad */
    #define US_D_OD_KDP_HACKY_REAR_OUTR_ADJ (0.20f)
#endif
    #define US_D_OD_KDP_LIMIT_ACCEPT_RANGE  (ME_TRUE)
    #if (US_D_OD_KDP_LIMIT_ACCEPT_RANGE == ME_TRUE)
        #define US_D_OD_KDP_ACCEPT_RANGE_MAX_CM    (300u) /* dir dist that 2D pt was created from */
    #endif
    #define US_D_OD_KDP_DEL_PT_ON_DIST  (ME_TRUE)
    #if (US_D_OD_KDP_DEL_PT_ON_DIST == ME_TRUE)
        #define US_D_OD_KDP_DEL_DIST_CM_FRNT     (15000.0f) /* Distance away from rear axis center to delete the dead points */
        #define US_D_OD_KDP_DEL_DIST_CM_REAR     (-15000.0f) /* Distance away from rear axis center to delete the dead points */
    #endif
#endif

/*
 * ************** END ALGORITHM SELECTION ON/OFF ************************
 */


/* Offset of three, because we can tolerate a few matching values,
 * without incurring much overhead.  Probably could go to -ONE actually.
 */
#define US_D_OD_MAX_PNTS_FOR_SEP_SORT (US_D_POINT_BUFFER_SIZE- THREE)

#ifdef  US_D_OD_HEIGHT_GEOM_BG_NOISE_TRACK
    #define US_D_OD_HEIGHT_GEOM_BG_NOISE_ALPHA (0.91f) /* takes 0.91 alpha takes about 1 second (25 cycles) to reach 10%, i.e. 0.91^(1sec/0.04sec==25) = 0.1 */
    #define US_D_OD_HEIGHT_GEOM_BG_NOISE_THRESH (21u) /* new points less than threshold will increment background counter (in dynamic mode, this would be different)*/
#endif
/* The data store for each groups point map and a simple sort index list */
static US_S_SnrPoint_t sSnrPnt[US_D_PHYS_GRP_MAX][US_D_POINT_BUFFER_SIZE];
static uint8_t uSnrPntSortIdx[US_D_PHYS_GRP_MAX][US_D_POINT_BUFFER_SIZE];
#if (US_D_OD_USE_FAST_SORT == ME_TRUE)
static uint8_t uSnrNewPntSortIdx[US_D_POINT_BUFFER_SIZE];
#endif

/* Buffer to store new points to incorporate in list.  Reused by each physical group. */
static US_S_SnrPoint_t sSnrNewPntBuff[US_D_POINT_BUFFER_SIZE];

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
/* Buffer to store kept points to incorporate in list.*/
static US_S_SnrPoint_t sSnrDeadPntLeftBuff[US_D_PHYS_GRP_MAX][US_D_DEAD_PNT_BUFF_SIZE];
static uint8_t uDeadPntRingLeftHead[US_D_PHYS_GRP_MAX] = {ZERO};
static uint8_t uDeadPntRingLeftTail[US_D_PHYS_GRP_MAX] = {ZERO};

static US_S_SnrPoint_t sSnrDeadPntRightBuff[US_D_PHYS_GRP_MAX][US_D_DEAD_PNT_BUFF_SIZE];
static uint8_t uDeadPntRingRightHead[US_D_PHYS_GRP_MAX] = {ZERO};
static uint8_t uDeadPntRingRightTail[US_D_PHYS_GRP_MAX] = {ZERO};

static bool_t  bIsFapaParkInActivePrev = ME_FALSE;
#endif

/* Object detection specific calculations */
static US_S_ObjDetCalcRec_t sObjDetCalcRec[US_D_MAX_NUM_SENSORS];
static US_E_App_Prndl_Status_e ePrndlPrev = US_PRNDL_PARK; /* Used in many places, updated at the end of echo location */
#if (US_D_OD_EGO_DIR_CHANGE_CONF_REDUCE == ME_TRUE)
static US_E_Dir_Ego_Motion_t ePrevDirOfActiveMotion = OD_DIR_EGO_MOT_UNKNOWN;
#endif

/* Single sensor detections/measurements for object detection */
#ifdef US_D_OD_ENABLE_2D_OBJECTS
static UssOD_MTT_Measurement_t sSingleSensorMeasList[US_D_MAX_NUM_SINGLE_SENSOR_MEAS];
static uint8_t nNumSingleSensorMeas;
#endif

/* Useful Indices */
static uint8_t uNumNewPnts = ZERO;
static uint8_t uNumCurPnts = ZERO;

#define US_D_OD_FLANK_FSM_IDLE      (0u)
#define US_D_OD_FLANK_FSM_CONFIRM   (1u)
#define US_D_OD_FLANK_FSM_ACTIVE    (2u)
#define US_D_OD_FLANK_FSM_INHIBIT   (3u)

/* For flanked confidence cap function */
#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
static uint8_t uFlankFSMState = US_D_OD_FLANK_FSM_IDLE;
static uint8_t uFlankConfirmCnt = ZERO;
static float32_t fCapConfActiveDist;
#endif
static float32_t fMotionTrkX = 0.0f;
static float32_t fMotionTrkY = 0.0f;

/* Group allow level logic data */
static US_E_Grp_Allow_Level_t eGrpAllowLevel[US_D_PHYS_GRP_MAX];
static UssMgr_E_GroupState_t g_prevGrpState[US_D_PHYS_GRP_MAX];
static uint8_t uSuppressCounter[US_D_PHYS_GRP_MAX]; //used for all allow levels
static uint8_t g_uSuppressNoisyThresh = US_D_OD_USE_NSK_CYCLES; //CVADAS debugger control var

//static uint32_t uCapConfActivationTime[US_D_PHYS_GRP_MAX] ={0};

#define US_D_OD_LONE_SNR_CHECK_OUTER (0)
#define US_D_OD_LONE_SNR_CHECK_INNER (1)
#define US_D_OD_LONE_SNR_CHECK_MAX   (4)
static uint8_t sLoneDirSnrCheckList[US_D_OD_LONE_SNR_CHECK_MAX][2] = {{US_D_SENSOR_FOR, US_D_SENSOR_FIR},
                                                                      {US_D_SENSOR_FOL, US_D_SENSOR_FIL},
                                                                      {US_D_SENSOR_ROL, US_D_SENSOR_RIL},
                                                                      {US_D_SENSOR_ROR, US_D_SENSOR_RIR}};

/* Dynamic Target garbage collection. aka Dead Point Sweeper */
#if (ME_TRUE == US_D_OD_DYN_TARG_DEAD_PNT_SWEEPER)
#define US_D_OD_SWEEPER_NUM_OF_MEAS_PNTS_MAX         (16u)
#define US_D_OD_SWEEPER_MIN_CONF_FOR_MEAS_PNT        (20u)
#define US_D_OD_SWEEPER_MIN_CRUISE_AGE_FOR_CONF_CAP  (4u)
#define US_D_OD_SWEEPER_CONF_CAP                     (40u)
#define US_D_OD_SWEEPER_X_DIST_MIN                   (7.0f) /* [cm] */
#define US_D_OD_SWEEPER_MAX_DIST                     (150.0f) /* [cm] */
#define US_D_OD_SWEEPER_MAX_DIST_SQ                  (US_D_OD_SWEEPER_MAX_DIST * US_D_OD_SWEEPER_MAX_DIST)

static void DynTargetDeadPntSweeper(uint8_t uGrpIdx);
#endif


/* 
 * Local functions
 */
static uint8_t UssOD_MatchPointWithNewList(uint8_t uGrpIdx, float32_t fX, float32_t fY);
static uint8_t UssOD_MatchPointWithPntList(uint8_t uGrpIdx, float32_t fX, float32_t fY, bool_t bEdgeBiasMatchAllowed);

#if (US_D_OD_USE_LONE_DIR_PNT_MATCHING == ME_TRUE)
static uint8_t UssOD_MatchMeasDistWithPntList(uint8_t uGrpIdx,
                                              uint8_t uDirSnrIdx,
                                              uint16_t uDirDist);
#endif

static void UssOD_AddNewPoints(uint8_t uGrpIdx);
static void UssOD_PerformPointDetection(uint8_t uGrpIdx);
static void UssOD_AgeAndAdjustOldPoints(uint8_t uGrpIdx, US_E_Grp_Allow_Level_t eGrpAllowLvl);
static void UssOD_SortAndMergeFinalList(uint8_t uGrpIdx);
#if (US_D_OD_USE_CR_DIST_KILL == ME_TRUE)
static void UssOD_AdjustTrackedPoints(uint8_t uGrpIdx);
#endif
static US_E_Grp_Allow_Level_t UssOD_GetAndUpdateGrpAllowLevel(uint8_t uGrpIdx, US_E_Dir_Ego_Motion_t eCurDirOfMotion);

static void UssOD_ComputeFinalXYAndStore(uint8_t uGrpIdx,
                                         uint8_t uDirSnrIdx,
                                         uint8_t uIndirSnrIdx,
                                         uint8_t uDirEchoIdx,
                                         uint16_t uSigStrength,
                                         uint16_t uDirDist,
                                         uint16_t uIndirDist,
                                         uint16_t uBaseDist,
                                         float32_t fPntAng,
#if US_D_OD_DEBUG_TRIANG == ME_TRUE
                                         float32_t fArea,         // For diagnostics only.
                                         float32_t fTriHeight,    // For diagnostics only.
#endif
                                         bool_t  bLargeObj,
                                         uint16_t uTriangType);

static bool_t UssOD_TriangEchoesDirToIndir(const US_S_SnrCalcs_t * psSnrCalcs,
                                           uint8_t uGrpIdx,
                                           uint8_t uDirSnrDataIdx,
                                           uint8_t uIndirSnrDataIdx,
                                           uint8_t uFovCalcIdx,
                                           bool_t  bIsLeftIndir);

static bool_t  UssOD_TriangEchoesDirToDir(const US_S_SnrCalcs_t * psSnrCalcs,
                                          uint8_t uGrpIdx,
                                          uint8_t uDirSnrDataIdx,
                                          uint8_t uIndirSnrDataIdx,
                                          uint8_t uFovCalcIdx,
                                          bool_t  bIsLeftIndir);

static bool_t UssOD_TriangEchoesTemporalDir(const US_S_SnrCalcs_t * psSnrCalcs,
                                            uint8_t uGrpIdx,
                                            uint8_t uDirSnrDataIdx);

static void UssOD_ConsiderLoneDirects(const US_S_SnrCalcs_t * psSnrCalcs,
                                      uint8_t uGrpIdx,
                                      uint8_t uDirSnrDataIdx);

static void UssOD_CreatePntInNewPntList(uint8_t uGrpIdx,
                                        uint8_t uDirEchoIdx,
                                        uint8_t uDirSnrIdx,
                                        uint8_t uIndirSnrIdx,
                                        uint16_t uTriangType,
                                        bool_t  bLargeObj,
                                        float32_t fX,
                                        float32_t fY,
                                        uint16_t uDirDist,
                                        uint16_t uIndirDist,
                                        uint16_t uBaseDist,
                                        uint16_t uSigStrength);

static void UssOD_MergePntToNewPntList(uint8_t uGrpIdx,
                                       uint8_t uMatchPntIdx,
                                       uint8_t uDirSnrIdx,
                                       uint8_t uIndirSnrIdx,
                                       uint16_t uTriangType,
                                       bool_t  bLargeObj,
                                       float32_t fX,
                                       float32_t fY,
                                       uint16_t uDirDist,
                                       uint16_t uIndirDist,
                                       uint16_t uBaseDist,
                                       uint16_t uSigStrength);

static void UssOD_MergePntToGrpPntList(uint8_t uGrpIdx,
                                       uint8_t uMatchPntIdx,
                                       uint8_t uDirSnrIdx,
                                       uint8_t uIndirSnrIdx,
                                       uint16_t uTriangType,
                                       bool_t  bLargeObj,
                                       float32_t fX,
                                       float32_t fY,
                                       uint16_t uDirDist,
                                       uint16_t uIndirDist,
                                       uint16_t uBaseDist,
                                       uint16_t uSigStrength,
                                       bool_t   bMergePnts);

static void UssOD_UpdateCacheDistanceCalcs(uint8_t uDirSnrDataIdx);

static void UssOD_UpdateCacheEntries(const US_S_SnrCalcs_t *psSnrCalcs,
                                     uint8_t  uDirSnrDataIdx,
                                     uint8_t uCurMeasUsageStat);

static bool_t UssOD_bIsPntInsideVehicle(float32_t fX, float32_t fY);

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
static void UssOD_CapConfForPntsWhenFlanked(uint8_t uGrpIdx);
static bool_t UssOD_bIsFlankingMitigationActive(void);
#endif

static bool_t UssOD_CheckFOV(float32_t fLeftPntAng,
                             float32_t fRightPntAng,
                             const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo,
                             bool_t bCheckInnerFov);


static uint8_t UssOD_CalcUpdatedPtConf(uint8_t uStartingConf, uint8_t uGrpIdx, uint16_t uSigStrength, uint16_t uDirDist, uint16_t uTriangType);
static uint8_t UssOD_CalcNewPtConf(uint8_t uStartingConf, uint8_t uGrpIdx, uint16_t uSigStrength, uint16_t uDirDist, uint16_t uTriangType);

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
static void UssOD_AddPntToDeadPntBuff(uint8_t uGrpIdx, US_S_SnrPoint_t * psSnrPnt);
static void UssOD_ResurrectDeadPnts(uint8_t uGrpIdx, uint8_t uTmpLeftOrRight, float32_t fAvgY);
static void UssOD_CheckForDeadPntResurrection(uint8_t uGrpIdx);
static void UssOD_UpdateOdometryForDeadPnts(uint8_t uGrpIdx,
                                            float32_t fCos,
                                            float32_t fSin,
                                            float32_t fDelX,
                                            float32_t fDelY);
#endif

#if (US_D_TD_UNIT_TESTING_MODE != ZERO)
void UssOD_TemporalDirUnitTest(uint8_t uDirSnrDataIdx,
                               bool_t bIsRearGroup,
                               uint16_t *uDirDist,
                               uint16_t *uIndirDist,
                               float32_t fDeltaX,
                               const US_S_Sensor_coordinate_t * pSnrCoord,
                               US_S_EchoCacheEntry_t *psEchoCacheEntry);
#endif

#if US_D_OD_USE_SQUARE_FOV_CHECK == ME_TRUE
static bool_t UssOD_CheckSquareFOV(bool_t bFovCheck,
                                   bool_t bIsLeftIndir,
                                   float32_t fLeftPntAng,
                                   float32_t uDirDist,
                                   float32_t uIndirDist,
                                   const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo);
#endif
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
/* temp debug variables */
uint16_t uTravelKillCounter = 0;
sint16_t siLastKill_X = 0;
sint16_t siAccuKill_X = 0;
uint8_t  uDBG_NumGrpPoints;
uint8_t  uDBG_ListPtIdx;
uint8_t  uDBG_InPtIdx;
uint8_t  uDBG_DeleteReason;
uint8_t  dbg_FapaSideTDDisable = 0;
#endif

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
volatile uint8_t uDbgCapConfFlag = ZERO;
#endif
#endif

/*===========================================================================
 * @brief Perform triangulation on each physical group of sensors and create point map for each group.
 *
 * @param
 * @param
 * @param
 * @return
 * @remarks Also calculates filtered distances arrays.
 */
/* ===========================================================================
 * Name:	 UssOD_PerformEchoLocation
 * Remarks:  DD-ID: {45DA229D-A66F-4662-9E0B-CFE0CE07498A}
 * Req.-ID: 13519880
 * Req.-ID: 13517239
 * Req.-ID: 15001705
 * Req.-ID: 17275272
 *
 * ===========================================================================*/
/*KPK-QAC Fix : Need to suppress, multiple branch/loop statements doesn't lead to any complexity */
void UssOD_PerformEchoLocation(void)/* PRQA S 2755 */
{
    uint8_t  uGrpIdx;
    const US_S_PlatPhysInfo_t * psPlatPhysInfo = UssCtrl_psGetPlatPhysInfo();
    US_E_Grp_Allow_Level_t eGroupAllowLevel;
    US_E_Dir_Ego_Motion_t eCurDirOfMotion = eGetCurVehDirOfMotion();

/* For 2D Object generation.  Clear number of total measurements for this direct sensor */
#ifdef US_D_OD_ENABLE_2D_OBJECTS
    nNumSingleSensorMeas = ZERO;
#endif

    /* Perform OD for each sensor physical group */
    for (uGrpIdx = ZERO; uGrpIdx < US_D_PHYS_GRP_MAX; uGrpIdx++)
    {
        if (psPlatPhysInfo->psSnrPhysGrp[uGrpIdx].uNumSnrsInGrp > ZERO) // Avoid processing groups that are not installed.
        {
            /* OD data pipeline.  Insert logic as needed */
            UssOD_ResetPntMeasStat(uGrpIdx);
            eGroupAllowLevel = UssOD_GetAndUpdateGrpAllowLevel(uGrpIdx, eCurDirOfMotion); //Update group allow level
            UssOD_AgeAndAdjustOldPoints(uGrpIdx, eGroupAllowLevel);   // Metrics of all points, measurement has occurred.  If a sending sensor, additional adjustments expected.
            if ( (eGroupAllowLevel != OD_GRP_SUPPRESS) && (eGroupAllowLevel != OD_GRP_CLR_N_SUPPRESS) )
            {
                UssOD_PerformPointDetection(uGrpIdx);           // Add any new points triangulated or otherwise, to old point or new point list.

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
                UssOD_CapConfForPntsWhenFlanked(uGrpIdx);       // Cap point confidence in vehicle corridor if flanked by points */
#endif

                UssOD_SortAndMergeFinalList(uGrpIdx);           // Sort and merge new and existing point for use on next cycle.
/* Jason Height function here. */
#ifdef US_D_OD_HEIGHT_GEOM_STATS
    #if (US_D_OD_HEIGHT_GEOM_DO_REAR_ONLY == ME_TRUE)
                if (US_D_PHYS_GRP_REAR == uGrpIdx)    {UssOD_UpdateHeightGeomStats(US_D_PHYS_GRP_REAR, uNumCurPnts);}
    #else
                UssOD_UpdateHeightGeomStats(uGrpIdx, uNumCurPnts);
    #endif    
#endif
/*
 * End height function.
 */
#if (US_D_OD_USE_CR_DIST_KILL == ME_TRUE)
    #if (US_D_OD_USE_CRDK_ONLY_FRONT == ME_TRUE)
                if (US_D_PHYS_GRP_FRNT == uGrpIdx)  {UssOD_AdjustTrackedPoints(US_D_PHYS_GRP_FRNT);}
    #else
                UssOD_AdjustTrackedPoints(uGrpIdx);
    #endif
#endif

/* Dynamic Target garbage collection. aka Dead Point Sweeper */
#if (ME_TRUE == US_D_OD_DYN_TARG_DEAD_PNT_SWEEPER)
                if (US_D_PHYS_GRP_FRNT == uGrpIdx) { DynTargetDeadPntSweeper(uGrpIdx); }
#endif
                UssOD_FindFiltGrpMinDistances(uGrpIdx); // Find Minimum distances to each sensor for Output
                UssOD_FindFiltMidDistances(uGrpIdx);    // Find/Calculate Mid sensor distances for Output
            }
        }
    }

    /* End of cycle storage of previous motion direction, when confirmed moving only */
#if (US_D_OD_EGO_DIR_CHANGE_CONF_REDUCE == ME_TRUE)
    if (eCurDirOfMotion != OD_DIR_EGO_MOT_UNKNOWN)
    {
        ePrevDirOfActiveMotion = eCurDirOfMotion;
    }
#endif

    if (US_D_OD_USE_ODOMETRY_FLAG_FUNC == ME_TRUE) //function call
    {
        UssOdo_ClearAccumulators();
    }

#ifdef US_D_OD_ENABLE_2D_OBJECTS
    /* Entry point for 2D Object algorithm.  To be computed once per direct sensor */
    if (nNumSingleSensorMeas != ZERO)
    {
        //UssOD_Update2dObjects(sSingleSensorMeasList, nNumSingleSensorMeas, psSnrMeasRec->uTimeStamp);
        UssOD_Update2dObjects(sSingleSensorMeasList, nNumSingleSensorMeas, psSnrMeasRec->uTimeStamp, UssOdo_psGetOdoAccumOut());
        nNumSingleSensorMeas = ZERO;
    }
#endif
    /* Update previous gear for this file, used elsewhere */
    ePrndlPrev = UssCom_F_GetVehPrndl();
}

/*===========================================================================
 * @brief Calculate the Area squared of a triangle, from three sides. (SSS)
 *
 * @param float32_t fHalfDistToSnr - Half of Side A of triangle
 * @param uint16_t uDistDir - Side B of triangle
 * @param uint16_t uDistIndir - Side C of triangle
 * @return float32_t Area square of a triangle.
 * @remarks Heron's formula:  S = (a + b + c) / 2.0;  A^2= S(S - a)(S - b)(S - c)
 */
/* ===========================================================================
 * Name:	 UssOD_CalcTriangAreaSq
 * Remarks:  DD-ID: {BDE2F6E3-F4AF-4810-80FA-19611CED9208}
 *Req.-ID: 15001645
 * ===========================================================================*/
float32_t UssOD_CalcTriangAreaSq(float32_t fHalfDistToSnr, uint16_t uDistDir, uint16_t uDistIndir)
{
    float32_t fHalfDistDir   = (float32_t) uDistDir * 0.5f;   // Heron's formula calls for 1/2 of each of the sides of the triangle.
    float32_t fHalfDistIndir = (float32_t) uDistIndir * 0.5f; // sides, of the triangle, to be added together.
    float32_t fS             =  fHalfDistToSnr + fHalfDistDir + fHalfDistIndir;
    float32_t fSa            = fS - (fHalfDistToSnr * 2.0f);
    float32_t fSb            = fS - (float32_t) uDistDir;
    float32_t fSc            = fS - (float32_t) uDistIndir;
    float32_t fAreaSquared   = fS * fSa * fSb * fSc;

    return fAreaSquared;
}

/*===========================================================================
 * @brief Match X,Y in with a point in new point buffer.
 *
 * @param uint8_t uSnrIdx
 * @param float32_t *fX
 * @param float32_t *fY
 * @return Return point index or US_D_OD_NO_MATCH_FOUND
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_MatchPointWithNewList
 * Remarks:  DD-ID: {BC1F536B-5432-473e-AB0C-71495F450D08}
 * ===========================================================================*/
static uint8_t UssOD_MatchPointWithNewList(uint8_t uGrpIdx, float32_t fX, float32_t fY)
{
    uint8_t   uPntIdx;
    uint8_t   uMatchPntIdx = US_D_OD_NO_MATCH_FOUND;
    UssMgr_E_GroupState_t   eGrpState = UssMgr_eGetGrpState(uGrpIdx);
    float32_t fListX;
    float32_t fListY;
    float32_t fMaxDistX;
    float32_t fMaxDistY;

    if (eGrpState != SYSMGR_GRPSTATE_OK ) //this covers NOISY and DEGRADED
    {
        fMaxDistX = US_D_OD_MATCH_DIST_X_NOISY;
        fMaxDistY = US_D_OD_MATCH_DIST_Y_NOISY;
    }
    else
    {
        fMaxDistX = US_D_OD_MATCH_DIST_X;
        fMaxDistY = US_D_OD_MATCH_DIST_Y;
    }

    for (uPntIdx = ZERO; uPntIdx < uNumNewPnts; uPntIdx++)
    {
        if (sSnrNewPntBuff[uPntIdx].uConf != ZERO) // Check if valid point
        {
            fListX = sSnrNewPntBuff[uPntIdx].fX;
            fListY = sSnrNewPntBuff[uPntIdx].fY;

            /* Check point bounds against point in new point buffer */
            if (   (fY > (fListY - fMaxDistY))
                && (fY < (fListY + fMaxDistY))
                && (fX > (fListX - fMaxDistX))
                && (fX < (fListX + fMaxDistX)))
            {
                uMatchPntIdx = uPntIdx;
                break;  // Found matching point
            }
        }
    }

    return uMatchPntIdx;
}


/*===========================================================================
 * @brief Match X,Y in with a point in sensor physical group point map.
 *
 * @param uint8_t uSnrIdx
 * @param float32_t *fX
 * @param float32_t *fY
 * @return Return point index or US_D_OD_NO_MATCH_FOUND
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_MatchPointWithPntList
 * Remarks:  DD-ID: {E62B8D1A-9975-43e7-831D-FEFF94C5C570}
 * ===========================================================================*/
/*KPK-QAC Fix : Need to suppress, multiple branch statements doesn't lead to any complexity */
static uint8_t UssOD_MatchPointWithPntList(uint8_t uGrpIdx, float32_t fX, float32_t fY, bool_t bEdgeBiasMatchAllowed)/* PRQA S 2755 */
{
    uint8_t   uListIdx;
    uint8_t   uPntIdx;
    uint8_t   uMatchPntIdx = US_D_OD_NO_MATCH_FOUND;
    uint8_t   uBiasedMatchPntIdx = US_D_OD_NO_MATCH_FOUND;
    UssMgr_E_GroupState_t   eGrpState = UssMgr_eGetGrpState(uGrpIdx);
    float32_t fListX;
    float32_t fListY;
    float32_t fMaxDistX;
    float32_t fMaxDistY;

    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    const US_S_SnrPoint_t * psSnrPnt;
    US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();

	if ((fY > psDrivingTubeInfo->fTubeEdgeY) || (fY < -psDrivingTubeInfo->fTubeEdgeY))
	{
		bEdgeBiasMatchAllowed = ME_FALSE;
	}
	
    if ((bool_t)ME_TRUE == bEdgeBiasMatchAllowed)
    {
        fMaxDistX = US_D_OD_MATCH_DIST_X_D_ONLY;
        fMaxDistY = US_D_OD_MATCH_DIST_Y_D_ONLY;
    }
    else if (eGrpState != SYSMGR_GRPSTATE_OK ) //this covers NOISY and DEGRADED
    {
        fMaxDistX = US_D_OD_MATCH_DIST_X_NOISY;
        fMaxDistY = US_D_OD_MATCH_DIST_Y_NOISY;
    }
    else
    {
        fMaxDistX = US_D_OD_MATCH_DIST_X;
        fMaxDistY = US_D_OD_MATCH_DIST_Y;
    }

    /* Scan point list in sorted order, highest .conf first.  Only need to search points with .conf > 0 */
    for (uListIdx = ZERO; uListIdx < uNumCurPnts; uListIdx++)
    {
        /* Get index and pointer to point data in sorted order */
        uPntIdx = puGrpSnrPntSortIdx[uListIdx];
        psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];

        if (psSnrPnt->uConf != ZERO)  // Check if valid point
        {
            fListX = psSnrPnt->fX;
            fListY = psSnrPnt->fY;

            /*
             * Normal window matching.
             * Check point bounds against point in point buffer
             */
            if (US_D_OD_NO_MATCH_FOUND == uMatchPntIdx)
            {
                if (   (fY > (fListY - fMaxDistY))
                    && (fY < (fListY + fMaxDistY))
                    && (fX > (fListX - fMaxDistX))
                    && (fX < (fListX + fMaxDistX)))
                {
                    uMatchPntIdx = uPntIdx; // Found matching point
                }
            }

            /*
             * Collect alternate match if primary fails.
             */
            if (   ((uint8_t)US_D_OD_NO_MATCH_FOUND == uBiasedMatchPntIdx)
                && ((bool_t)ME_TRUE == bEdgeBiasMatchAllowed))
            {
                if ((fListY > psDrivingTubeInfo->fTubeEdgeY) || (fListY < -psDrivingTubeInfo->fTubeEdgeY))
                {
                    if (   (fX > (fListX - US_D_OD_MATCH_DIST_BIAS_X_D_ONLY))
                        && (fX < (fListX + US_D_OD_MATCH_DIST_BIAS_X_D_ONLY)))
                    {
                        if (fListY < 0.0f)  // Point on Right
                        {
                            /* Check point bounds against point in new point buffer */
                            if (   (fY < (fListY + US_D_OD_MATCH_DIST_BIAS_Y_D_ONLY))
                                && (fY > fListY))
                            {
                                uBiasedMatchPntIdx = uPntIdx; // Found matching point, but keep running
                            }
                        }
                        else // Point on Left
                        {
                            /* Check point bounds against point in point buffer */
                            if (   (fY > (fListY - US_D_OD_MATCH_DIST_BIAS_Y_D_ONLY))
                                && (fY < fListY)) // 1cm
                            {
                                uBiasedMatchPntIdx = uPntIdx; // Found matching point, but keep running
                            }
                        }
                    }
                }
            }
        }

        /* Matches found (or disable) for both.  No need to continue loop */
        if (   (US_D_OD_NO_MATCH_FOUND != uMatchPntIdx)
            && (   (ME_FALSE == (bool_t) bEdgeBiasMatchAllowed)
                || (US_D_OD_NO_MATCH_FOUND != uBiasedMatchPntIdx)))
        {
            break;
        }
    }

    /* Arbitration to select best point match. */
    if (US_D_OD_NO_MATCH_FOUND != uMatchPntIdx)
    {
        psSnrPnt = &sSnrPnt[uGrpIdx][uMatchPntIdx];

        /* If the match was a latched triangulation it takes presidence over bias matching */
        if ((psSnrPnt->uPntStat & US_D_PNTSTAT_TRIANG_LATCHED_BITS) != ZERO)
        {
            uBiasedMatchPntIdx = (uint8_t)US_D_OD_NO_MATCH_FOUND;
        }
    }

    if (((bool_t)ME_TRUE == bEdgeBiasMatchAllowed) && (uBiasedMatchPntIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND))
    {
        uMatchPntIdx = uBiasedMatchPntIdx | US_D_OD_BIAS_MATCH_FLAG;
    }

    return uMatchPntIdx;
}

/*===========================================================================
 * @brief Match direct distance in with a point in sensor physical group point map.
 * DD-ID: {6F5EA57D-3D0A-49a6-A074-30E2D1A0AF36}
 * @param uint8_t uSnrIdx
 * @param uint16_t uDirDist - Direct distance to scan for match.
 * @return Return point index or US_D_OD_NO_MATCH_FOUND
 * @remarks
 */

#if (US_D_OD_USE_LONE_DIR_PNT_MATCHING == ME_TRUE)
/* ===========================================================================
 * Name: UssOD_MatchMeasDistWithPntList	
 * Remarks:  DD-ID: {94F7451A-3EF2-4717-8695-FAB3B155EA04} 
 * ===========================================================================*/

static uint8_t UssOD_MatchMeasDistWithPntList(uint8_t uGrpIdx,
                                              uint8_t uDirSnrIdx,
                                              uint16_t uDirDist)
{
#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
    bool_t bIsFlankingMitigationActive = UssOD_bIsFlankingMitigationActive();
#endif
    bool_t    bSkipMatchingInTubePnts;
    uint8_t   uListIdx;
    uint8_t   uPntIdx;
    uint8_t   uMatchPntOutsideTriangIdx = US_D_OD_NO_MATCH_FOUND;
    uint8_t   uMatchPntOutsideAnyIdx    = US_D_OD_NO_MATCH_FOUND;
    uint8_t   uMatchPntInsideTriangIdx  = US_D_OD_NO_MATCH_FOUND;
    uint8_t   uMatchPntIdx              = US_D_OD_NO_MATCH_FOUND;
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    float32_t fSensorX;             // Location of sensor in vehicle coord system
    float32_t fSensorY;
    float32_t fListX;               // Location of group list point in vehicle coord system
    float32_t fListY;
    float32_t fDeltaX;              // Delta from sensor loc to group point loc
    float32_t fDeltaY;
    float32_t fDeltaTmp;            // Used for temp calculation.
    float32_t fSqrPntDist;          // Square of group point distance.
    float32_t fSqrDirDistRangeLow;  // Direct measurement distance low range squared.
    float32_t fSqrDirDistRangeHigh; // Direct measurement distance high range squared.

    float32_t fOutsideTriangMaxDelY =    0.0f; /* Highest delta in Y, towards outside, previously traingulated points only. */
    float32_t fOutsideAnyMatch      =    0.0f; /* Highest delta in Y, towards outside, any point. */
    float32_t fInsideTriangMaxDelY  = -500.0f; /* Lowest delta in Y, towards inside, previously traingulated points only. */

    const US_S_SnrPoint_t * psSnrPnt;
    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;
    const US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }
    if(pSnrCoord == NULL)
    {
        return uMatchPntIdx;   // Check if data exists.  Early return for functional safety.
    }

    /* Calculate range low (squared) */
    fDeltaTmp = ((float32_t) uDirDist - US_D_OD_MATCH_RANGE_MATCH_DIST);
    if (fDeltaTmp < 0.0f)
    {
        fDeltaTmp = 0.0f;
    }
    fSqrDirDistRangeLow  = fDeltaTmp * fDeltaTmp;

    /* Calculate range high (squared) */
    fDeltaTmp = ((float32_t) uDirDist + US_D_OD_MATCH_RANGE_MATCH_DIST);
    fSqrDirDistRangeHigh = fDeltaTmp * fDeltaTmp;

    fSensorX = pSnrCoord[uDirSnrIdx].fX;
    fSensorY = pSnrCoord[uDirSnrIdx].fY;

    /* Scan point list, excluding low conf points, and find the best match based on abs(y) */
    for (uListIdx = ZERO; uListIdx < uNumCurPnts; uListIdx++)
    {
        /* Get index and pointer to point data in sorted order */
        uPntIdx = puGrpSnrPntSortIdx[uListIdx];
        psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];

        if (   (psSnrPnt->uConf >= US_D_OD_MATCH_RANGE_MIN_CONF))  // Check if valid point
        {
            fListX = psSnrPnt->fX;
            fListY = psSnrPnt->fY;

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
            /*
             * Don't match DDs in tube when flanking mitigation is active.
             */
            bSkipMatchingInTubePnts = (bool_t)(   ((bool_t)ME_TRUE == bIsFlankingMitigationActive)
                                               && (fListY > -psDrivingTubeInfo->fTubeEdgeY)
                                               && (fListY <  psDrivingTubeInfo->fTubeEdgeY));

#else
            bSkipMatchingInTubePnts = ME_FALSE;
#endif

            if (ME_FALSE == bSkipMatchingInTubePnts)
            {
                /* Calculate distance squared from point to sensor. */
                fDeltaX = fListX - fSensorX;
                fDeltaY = fListY - fSensorY;
                fSqrPntDist = fDeltaX * fDeltaX + fDeltaY * fDeltaY;

                // Compare distance (range) of direct measurement arc to our point distance from sensor.
                if (   (fSqrPntDist > fSqrDirDistRangeLow)
                    && (fSqrPntDist < fSqrDirDistRangeHigh))
                {
                    /* If sensor is on RIGHT, negative Y, side of the vehicle, negate delta for max check.
                     * After negation, positive delta shall always indicate point is towards the outside of
                     * the vehicle (wrt sensor location).
                     */
                    if (fSensorY < 0.0f)
                    {
                        fDeltaY = -fDeltaY;
                    }

                    /* If point was previously triangulated */
                    if ((psSnrPnt->uPntStat & US_D_PNTSTAT_TRIANG_LATCHED_BITS) != ZERO)  /* Under the condition that the point was previously triangulated. */
                    {
                        /* Check if towards outside of the vehicle tube wrt sensor location*/
                        if (fDeltaY >= 0.0f)
                        {
                            /* Match with the point with the highest delta Y. (Towards outer edge of vehicle.)
                             * So we are looking for the highest delta Y
                             */
                            if (fOutsideTriangMaxDelY <= fDeltaY)
                            {
                                fOutsideTriangMaxDelY = fDeltaY;
                                uMatchPntOutsideTriangIdx = uPntIdx;
                            }
                        }
                        else /* Point is towards inside of vehicle wrt sensor location. */
                        {
                            /* Match with the point with the highest |Y| coordinate.
                             * (Towards outside of vehicle, but inside sensor location.)
                             * So we are looking for the lowest Y, which will be negative
                             */
                            if (fInsideTriangMaxDelY <= fDeltaY)
                            {
                                fInsideTriangMaxDelY = fDeltaY;
                                uMatchPntInsideTriangIdx = uPntIdx;
                            }
                        }
                    }
                    else /* Point was previously untriangulated */
                    {
                        /* Check if towards outside of the vehicle tube wrt sensor,
                         * Lowest arbitration level.
                         * Untriangulated points towards inside of vehicle will not match anything,
                         * and ultimately new point will be placed at sensor normal angle if no other
                         * match is found.
                         */
                        if (fDeltaY >= 0.0f)
                        {
                            /* Match with the point with the highest |Y| coordinate. (Towards outer edge of vehicle and beyond.) */
                            if (fOutsideAnyMatch <= fDeltaY)
                            {
                                fOutsideAnyMatch = fDeltaY;
                                uMatchPntOutsideAnyIdx = uPntIdx;
                            }
                        }
                        /*
                         * No else.  Untriangulated points towards center of vehicle wrt sensor location
                         * will not match and if no other points are matched, will result in a lone
                         * that is placed at sensor normal angle.
                         */
                    }
                }
            }
        }
    }

    /* Perform Arbitration */
    if (uMatchPntOutsideTriangIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND)     /* #1 priority, triang point towards outside of vehicle */
    {
        uMatchPntIdx = uMatchPntOutsideTriangIdx;
    }
    else if (uMatchPntInsideTriangIdx !=  (uint8_t)US_D_OD_NO_MATCH_FOUND)  /* #2 priority, triang point towards inside of vehicle */
    {
        uMatchPntIdx = uMatchPntInsideTriangIdx;
    }
    else if (uMatchPntOutsideAnyIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND)    /* #3 priority, un-triang point  towards outside of vehicle */
    {
        uMatchPntIdx = uMatchPntOutsideAnyIdx;
    }
    else
    {
        /* Failed to find a suitable match. Do nothing. */
    }

    return uMatchPntIdx;
}
#endif
/*===========================================================================
 * @brief Overrides or caps the confidence for the point, 
 *          split between a brand new point and updating a point
 * @param
 * @param
 * @param
 * @return
 */
/* ===========================================================================
 * Name: UssOD_CalcNewPtConf	
 * Remarks:  DD-ID: {6DB9DDCB-92BC-4959-871C-C8373E1E858E} 
 * ===========================================================================*/
static uint8_t UssOD_CalcNewPtConf(uint8_t uStartingConf, uint8_t uGrpIdx, uint16_t uSigStrength, uint16_t uDirDist, uint16_t uTriangType)
{
    uint8_t     uSigStrShift;
    uint16_t    uDistFarPnt;
    uint8_t     uFarPntConfCap;
    uint8_t     uNormalConfCap;
    uint8_t     uRetConf = US_D_OD_DEFAULT_CONF;
    uint8_t     uConfCap;
    uint16_t    uConfInc;
    UssMgr_E_GroupState_t eGrpState = UssMgr_eGetGrpState(uGrpIdx);

    /* Gather the calibration values */
    if (eGrpState != SYSMGR_GRPSTATE_OK ) //this covers NOISY and DEGRADED
    {
        //use slower values
        uSigStrShift = US_D_OD_INITIAL_SIG_STR_SHFT__SLOWMODE; //replace with US_Calibration function
        uDistFarPnt = US_D_OD_MIN_DIST_OF_FAR_PNT;
        uFarPntConfCap = US_D_OD_MAX_CONF_FOR_FAR_PNTS__SLOWMODE;
        uNormalConfCap = US_D_OD_PNT_INITIAL_MAX_CONF__SLOWMODE;
    }
    else
    {
        //use normal values
        uSigStrShift = US_D_OD_INITIAL_SIG_STR_SHFT; //replace with US_Calibration function
        uDistFarPnt = US_D_OD_MIN_DIST_OF_FAR_PNT;
        uFarPntConfCap = US_D_OD_MAX_CONF_FOR_FAR_PNTS;
        uNormalConfCap = US_D_OD_PNT_INITIAL_MAX_CONF;
    }

    if ((uTriangType & US_D_PNTSTAT_DIRECT_ONLY) == US_D_PNTSTAT_DIRECT_ONLY)
    {
        uFarPntConfCap >>= TWO;
    }

    /* Boost initial confidence with a fraction of the signal strength */
    uConfInc = (uSigStrength >> uSigStrShift);

    /* capping the confidence */
    if (uDirDist >=  uDistFarPnt)
    {
        uConfCap = uFarPntConfCap;
    }
    else
    {
        uConfCap = uNormalConfCap;
    }
    /* final confidence calculation */
    if ((uStartingConf + uConfInc) > uConfCap) // This limiter deserves more thought.
    {
        uRetConf = uConfCap;
    }
    else
    {
        uRetConf = (uint8_t) (uStartingConf + uConfInc);
    }

    return uRetConf;
}


/* ===========================================================================
 * Name: UssOD_CalcUpdatedPtConf	
 * Remarks:  DD-ID:{FF8B4E1B-AFEF-4c2f-A99A-AC1E221EA8F3}
 * ===========================================================================*/
static uint8_t UssOD_CalcUpdatedPtConf(uint8_t uStartingConf, uint8_t uGrpIdx, uint16_t uSigStrength, uint16_t uDirDist, uint16_t uTriangType)
{
    (void) uTriangType;

    uint8_t     uSigStrShift;
    uint16_t    uDistFarPnt;
    uint8_t     uFarPntConfCap;
    uint8_t     uNormalConfCap;
    uint8_t     uConfBoost;
    uint8_t     uRetConf = US_D_OD_DEFAULT_CONF;
    uint8_t     uConfCap;
#if (US_D_USE_FAPA_API == ME_TRUE)
    bool_t      bIsFapaActive = ME_FALSE;
#endif
    uint16_t    uConfInc;
    float32_t fVehSpeed_kph = 0.0f;
    UssMgr_E_GroupState_t   eGrpState = UssMgr_eGetGrpState(uGrpIdx);
    fVehSpeed_kph = UssCom_F_GetVehSpeed();

    /* Gather the calibration values */
    if (eGrpState != SYSMGR_GRPSTATE_OK ) //this covers NOISY and DEGRADED
    {
        //use slower values
        uSigStrShift = US_D_OD_SIG_STRENGTH_SHFT__SLOWMODE; //replace with US_Calibration function
        uDistFarPnt = US_D_OD_MIN_DIST_OF_FAR_PNT;
        uFarPntConfCap = US_D_OD_MAX_CONF_FOR_FAR_PNTS__SLOWMODE;
        uNormalConfCap = US_D_OD_PNT_MAX_CONF;
#ifdef US_D_OD_HEIGHT_GEOM_BG_NOISE_TRACK
        uConfBoost = g_PntConfBoostSlowMode;  /* TBD: make dynamic based on g_fBackgroundNoiseLevel */
#else 
        uConfBoost = US_D_OD_PNT_CONF_BOOST__SLOWMODE;
#endif

        if (fVehSpeed_kph > 0.5f)
        {
            uConfBoost = US_D_OD_PNT_CONF_BOOST__SLOWMODE + US_D_OD_PNT_CONF_ADDER_SLOWMODE;
        }

    
#if (US_D_USE_FAPA_API == ME_TRUE)
        TbAP_DriveAssistStatOut_t sDriveAssistStatOut;
        SigMgr_PpDriveAssistStatOut_TbAP_DriveAssistStatOut_t_Get(&sDriveAssistStatOut);
        /* Do not perform mitigation in FAPA mode. */
        bIsFapaActive = (TeAutoPark_e_AutoParkStatus_Active == sDriveAssistStatOut.IeAP_e_AutoParkStatus)
                        && (TeAP_e_AlgoState_Park_In == sDriveAssistStatOut.IeAP_e_AlgoState);
#endif

#if (OD_D_USE_FAPA_FEATURE_WITH_NO_API == ME_FALSE)
        if (ME_TRUE == bIsFapaActive)
#endif
        {
            uConfBoost += US_D_OF_PNT_CONF_FAPA_SLOWMODE_XTRA_BOOST;
        }
    }
    else
    {
        //use normal values
        uSigStrShift = US_D_OD_SIG_STRENGTH_SHFT; //replace with US_Calibration function
        uDistFarPnt = US_D_OD_MIN_DIST_OF_FAR_PNT;
        uFarPntConfCap = US_D_OD_MAX_CONF_FOR_FAR_PNTS;
        uNormalConfCap = US_D_OD_PNT_MAX_CONF;
        uConfBoost = US_D_OD_PNT_CONF_BOOST;
    }

#if (US_D_OD_SLOW_N_CAP_NON_DIR_INDIR == ME_TRUE)
    if ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG) == ZERO)
    {
        uConfBoost = FIVE;
        uFarPntConfCap >>= TWO;
    }
#endif


    /* Boost initial confidence with a fraction of the signal strength */
    uConfInc = uConfBoost + (uSigStrength >> uSigStrShift);

    /* capping the confidence */
    if (uDirDist >=  uDistFarPnt)
    {
        uConfCap = uFarPntConfCap;
    }
    else
    {
        uConfCap = uNormalConfCap;
    }

    /* final confidence calculation */
    if ((uStartingConf + uConfInc) > uConfCap) // This limiter deserves more thought.
    {
        uRetConf = uConfCap;
    }
    else
    {
        uRetConf = (uint8_t) (uStartingConf + uConfInc);
    }

    return uRetConf;
}

/*===========================================================================
 * @brief TBD
 *
 * @param
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_MergePntToGrpPntList
 * Remarks:  DD-ID: {9D95F34A-9542-4092-AE8F-34DACD84C4F5}
 * Traceability to source Code: Req.-ID: 17275275
 * ===========================================================================*/
static void UssOD_MergePntToGrpPntList(uint8_t uGrpIdx,
                                       uint8_t uMatchPntIdx,
                                       uint8_t uDirSnrIdx,
                                       uint8_t uIndirSnrIdx,
                                       uint16_t uTriangType,
                                       bool_t  bLargeObj,
                                       float32_t fX,
                                       float32_t fY,
                                       uint16_t uDirDist,
                                       uint16_t uIndirDist,
                                       uint16_t uBaseDist,
                                       uint16_t uSigStrength,
                                       bool_t   bMergePnts)
{
    uint8_t uConf;
    uint8_t uEchoCalcFlags;
    bool_t  bConfCap;
#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
    bool_t bIsFlankingMitigationActive = UssOD_bIsFlankingMitigationActive();
#endif

    US_S_SnrPoint_t * psMatchedPoint = &sSnrPnt[uGrpIdx][uMatchPntIdx];
    const US_S_SnrCalcs_t * psSnrCalcs = UssMgr_psGetCurSnrCalcsRec();

	if ((bool_t)ME_TRUE == bMergePnts)
	{
		/* Combine old and new points */
		psMatchedPoint->fX = (fX * 0.3f + psMatchedPoint->fX * 0.7f); // * US_D_OD_SIMPLE_AVERAGE;
		psMatchedPoint->fY = (fY * 0.3f + psMatchedPoint->fY * 0.7f); // * US_D_OD_SIMPLE_AVERAGE;
	}

    /* Reset decrement rate exponent to default */

    uEchoCalcFlags = psSnrCalcs->sSnrCalcRec[uDirSnrIdx].sEchoCalc[ZERO].uEchoCalcFlags;

    // Check if fake NFD.
    if (   ((uEchoCalcFlags & US_D_CALCFLAG_FAKE) != ZERO)
        && ((uEchoCalcFlags & US_D_CALCFLAG_HIGH_CONF) != ZERO))
    {
        psMatchedPoint->uDecRate     = US_D_OD_INIT_FAKENFD_NOMEAS_DEC_RATE_EXP;
    }
    else
    {
#ifdef US_D_USE_SLOW_CONF_APP_3
        psMatchedPoint->uDecRate = US_D_OD_NOMEAS_DEC_RATE_EXP;
#else
        psMatchedPoint->uDecRate     = US_D_OD_INIT_NOMEAS_DEC_RATE_EXP;
#endif
    }

    // Check if fake NFD.  If so, confidence forced at max.
    if (   ((uEchoCalcFlags & US_D_CALCFLAG_FAKE) != ZERO)
        && ((uEchoCalcFlags & US_D_CALCFLAG_HIGH_CONF) != ZERO))
    {
        uConf = US_D_OD_PNT_MAX_CONF;
    }
    else
    {
        /* Adjust confidence, limit confidence, and store confidence*/
        uConf = UssOD_CalcUpdatedPtConf(psMatchedPoint->uConf, uGrpIdx, uSigStrength, uDirDist, uTriangType);

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
        /*
         * Stop anything but DIs from building confidence in driving tube when in tunnel mitigation mode.
         */
        US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();

        if (   ((bool_t) ME_TRUE == bIsFlankingMitigationActive)
            && ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG) == ZERO))
        {
            /* Only in middle!! */
            if ((fY > -psDrivingTubeInfo->fTubeEdgeY) && (fY < psDrivingTubeInfo->fTubeEdgeY))
            {
                /* Override calculated conf to offer no increase in tube */
                uConf = psMatchedPoint->uConf;
            }
        }
#endif
    }

    /* We have previously been marked with US_D_PNTSTAT_MEASURED when added to the new points list. */
    /* Set out point creation flags */
    uTriangType |= psMatchedPoint->uPntStat; /* Merge in old point status to candidate status */

    /* Confidence capped latch check */
    bConfCap = ((uTriangType & US_D_PNTSTAT_CONF_CAPPED_LTCH) != ZERO);

    if ((bool_t)ME_TRUE == bConfCap)  /* Currently conf capped */
    {
        if (uDirDist > US_D_OD_CONF_CAP_LOCKOUT_RANGE)
        {
            /* Conditions to unlatch conf cap */
            if ((uTriangType & US_D_PNTSTAT_DIR_DIR_TRING_LTCH) != ZERO) /* If DD ever measured for this point. */
            {
                bConfCap = ME_FALSE; /* Remove caching latch */
            }
            else{/*do nothing*/}
#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
            /* Release conf camp when performing flanking mitigation to
             * allow objects in mitigation that have been triangulated
             * in a single measurement, to grow in conf.
             */
            if (   ((bool_t) ME_TRUE == bIsFlankingMitigationActive)
                && ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG_LTCH) != ZERO))
            {
                bConfCap = ME_FALSE; /* Remove caching latch */
            }
            else{/*do nothing*/}
#endif

#if (US_D_OD_USE_DI_MULTI_SNR_CCAP_RELEASE == ME_TRUE)
            if (   ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG_LTCH) != ZERO)  /* Has been measured by DI before */
                && (psMatchedPoint->uDirSnrIdx != uDirSnrIdx))                              /* and from a different sensor */
            {
                bConfCap = ME_FALSE; /* Remove caching latch */
            }
#endif

        }
        else
        {
            bConfCap = ME_FALSE; /* Remove caching latch because very close*/
        }
    }

    /* If still currently conf capped */
    if ((bool_t) ME_TRUE == bConfCap)
    {
        if ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG) != ZERO)  /* This is a DI triang type */
        {
            if (uConf > US_D_OD_MAYBE_CONF_CAP_DI)
            {
                uConf = US_D_OD_MAYBE_CONF_CAP_DI;
            }
            else{/*do nothing*/}
        }
        else
        {
            if (uConf > US_D_OD_MAYBE_CONF_CAP_NOT_DI)
            {
                uConf = US_D_OD_MAYBE_CONF_CAP_NOT_DI;
            }
            else{/*do nothing*/}
        }
    }
    else
    {
        uTriangType &= ~((uint16_t)US_D_PNTSTAT_CONF_CAPPED_LTCH); /* Remove caching latch for current meas*/
    }

    /*
     * Update point status with triangle type
     */
    uTriangType |= (US_D_PNTSTAT_MEASURED | US_D_PNTSTAT_MEAS_MULT_CYCLES);

    if (   (psMatchedPoint->uDirSnrIdx != uDirSnrIdx) /* from a different sensor */
        || ((psMatchedPoint->uPntStat & US_D_PNTSTAT_MEASURED) != ZERO)) /* Or again on this cycle.  Probably redundant. */
    {
        uTriangType |= US_D_PNTSTAT_MEAS_MULT_SNRS;
    }
    else{/*do nothing*/}

    psMatchedPoint->uPntStat = uTriangType;

    psMatchedPoint->uConf = uConf;

    /* Reset cruising age since we measured the point again */
    psMatchedPoint->uCruisingAge = ZERO;

    psMatchedPoint->fAccuX = 0.0f;
    psMatchedPoint->fAccuY = 0.0f;

    if (bLargeObj == (bool_t)ME_TRUE)
    {
        psMatchedPoint->uHeight = TWO;
    }
    else
    {
        psMatchedPoint->uHeight = ONE;
    }

    /* For diagnostics purposes only. @TODO jp - consider putting #def around */
    psMatchedPoint->uIndirDist = uIndirDist;

    if (uDirDist != ZERO) // Safety.  I can't see how this could occur.
    {
        psMatchedPoint->uLowDirDist = uDirDist;
    }
    else{/*do nothing*/}

    /*
     * Store last measuring sensors for min and mid distance calculations -
     */
    psMatchedPoint->uDirSnrIdx  = uDirSnrIdx;
    psMatchedPoint->uIndirSnrIdx  = uIndirSnrIdx;

    /* Update sector Idx, if necessary */
    psMatchedPoint->uBaseDist = uBaseDist;
#if (USS_FEATURE_ZONE_INFO == US_STD_ON)
    psMatchedPoint->uSectorIdx = UssOD_uFindSectorIdx(fX, fY, psMatchedPoint->uSectorIdx);
#endif
}

/*===========================================================================
 * @brief TBD
 *
 * @param
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_MergePntToNewPntList
 * Remarks:  DD-ID: {4D0436C3-7FBF-4583-BD8A-22AC0870C092}
 * Traceability to source Code: Req.-ID: 17275275
 * Req.-ID: 15002839
 * Req.-ID: 17275275
 * Req.-ID: 17275282
 * Req.-ID: 13420481
 * Req.-ID: 13520021
 * Req.-ID: 13520023
 * Req.-ID: 13520032
 * Req.-ID: 13520039
 * Req.-ID: 13520042
 * Req.-ID: 13520045
 * ===========================================================================*/
static void UssOD_MergePntToNewPntList(uint8_t uGrpIdx,
                                       uint8_t uMatchPntIdx,
                                       uint8_t uDirSnrIdx,
                                       uint8_t uIndirSnrIdx,
                                       uint16_t uTriangType,
                                       bool_t  bLargeObj,
                                       float32_t fX,
                                       float32_t fY,
                                       uint16_t uDirDist,
                                       uint16_t uIndirDist,
                                       uint16_t uBaseDist,
                                       uint16_t uSigStrength)
{
    /*KPK-QAC Fix : Need to suppress,Required for compilation */
    (void) uGrpIdx;/* PRQA S 3119 */

    uint8_t uConf;
    bool_t  bConfCap;
    US_S_SnrPoint_t * psMatchedPoint = &sSnrNewPntBuff[uMatchPntIdx];


    /* Combine points */
    psMatchedPoint->fX = (fX + psMatchedPoint->fX) * US_D_OD_SIMPLE_AVERAGE;
    psMatchedPoint->fY = (fY + psMatchedPoint->fY) * US_D_OD_SIMPLE_AVERAGE;

    /* Adjust confidence, limit confidence, and store confidence and reset cruising age */
    uConf = UssOD_CalcUpdatedPtConf(psMatchedPoint->uConf, uGrpIdx, uSigStrength, uDirDist, uTriangType);

    /* We have previously been marked with US_D_PNTSTAT_MEASURED when added to the new points list. */
    /* Set out point creation flags */
    uTriangType |= psMatchedPoint->uPntStat; /* Merge in old point status to candidate status */

    /* Confidence capped latch check */
    bConfCap = ((uTriangType & US_D_PNTSTAT_CONF_CAPPED_LTCH) != ZERO);

    if ((bool_t)ME_TRUE == bConfCap)  /* Currently conf capped */
    {
        if (uDirDist > US_D_OD_CONF_CAP_LOCKOUT_RANGE)
        {
            /* Conditions to unlatch conf cap */
            if ((uTriangType & US_D_PNTSTAT_DIR_DIR_TRING_LTCH) != ZERO) /* If DD ever measured for this point. */
            {
                bConfCap = (bool_t)ME_FALSE; /* Remove caching latch */
            }
            else{/*do nothing*/}

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
            /* Release conf camp when performing flanking mitigation to
             * allow objects in mitigation that have been triangulated
             * in a single measurement, to grow in conf.
             */
            bool_t bIsFlankingMitigationActive = UssOD_bIsFlankingMitigationActive();
            if (   ((bool_t) ME_TRUE == bIsFlankingMitigationActive)
                && ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG_LTCH) != ZERO))
            {
                bConfCap = ME_FALSE; /* Remove caching latch */
            }
            else{/*do nothing*/}
#endif

#if (US_D_OD_USE_DI_MULTI_SNR_CCAP_RELEASE == ME_TRUE)
            if (   ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG_LTCH) != ZERO)  /* Has been measured by DI before */
                && (psMatchedPoint->uDirSnrIdx != uDirSnrIdx))                              /* and from a different sensor */
            {
                bConfCap = ME_FALSE; /* Remove caching latch */
            }
#endif

        }
        else
        {
            bConfCap = (bool_t)ME_FALSE; /* Remove caching latch because very close*/
        }
    }

    /* If still currently conf capped */
    if (ME_TRUE == bConfCap)
    {
        if ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG) != ZERO)  /* This is a DI triang type */
        {
            if (uConf > US_D_OD_MAYBE_CONF_CAP_DI)
            {
                uConf = US_D_OD_MAYBE_CONF_CAP_DI;
            }
            else{/*do nothing*/}
        }
        else
        {
            if (uConf > US_D_OD_MAYBE_CONF_CAP_NOT_DI)
            {
                uConf = US_D_OD_MAYBE_CONF_CAP_NOT_DI;
            }
            else{/*do nothing*/}
        }
    }
    else
    {
        uTriangType &= ~((uint16_t)US_D_PNTSTAT_CONF_CAPPED_LTCH); /* Remove caching latch for current meas*/
    }

    /* After a point has been merged in once one on a new point cycle, it is measured by multiple sensors
     * and has already been marked as measued by new point add method.
     */
    uTriangType |= (US_D_PNTSTAT_MEASURED | US_D_PNTSTAT_MEAS_MULT_SNRS);
    /*
     * Update point status with triangle type
     */
    psMatchedPoint->uPntStat = uTriangType;

    psMatchedPoint->uConf = uConf;

    /* Reset cruising age since we measured the point again */
    psMatchedPoint->uCruisingAge = ZERO;
    psMatchedPoint->fAccuX = 0.0f;
    psMatchedPoint->fAccuY = 0.0f;

    /* For diagnostics purposes only. @TODO jp - consider putting #def around */
    psMatchedPoint->uIndirDist = uIndirDist;

    /* Calculations for min/mid distance values */
    //if (uDirDist < psMatchedPoint->uLowDirDist)
    {
        if (uDirDist != ZERO)
        {
          
            psMatchedPoint->uLowDirDist = uDirDist;
        }
        else{/*do nothing*/}
    }

    if (bLargeObj == (bool_t)ME_TRUE)
    {
        psMatchedPoint->uHeight = TWO;
    }
    else
    {
        psMatchedPoint->uHeight = ONE;
    }

    /* Store last measuring sensors for min and mid distance calculations */
    psMatchedPoint->uDirSnrIdx = uDirSnrIdx;
    psMatchedPoint->uIndirSnrIdx = uIndirSnrIdx;

    /* Update sector Idx, if necessary */

    psMatchedPoint->uBaseDist = uBaseDist;
#if (USS_FEATURE_ZONE_INFO == US_STD_ON)
    psMatchedPoint->uSectorIdx = UssOD_uFindSectorIdx(fX, fY, psMatchedPoint->uSectorIdx);
#endif
}

/*===========================================================================
 * @brief TBD
 *
 * @param
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_CreatePntInNewPntList
 * Remarks:  DD-ID: {6D03DC20-4F6F-4720-A04F-C970AD38AA62}
 * ===========================================================================*/
static void UssOD_CreatePntInNewPntList(uint8_t uGrpIdx,
                                        uint8_t uDirEchoIdx,
                                        uint8_t uDirSnrIdx,
                                        uint8_t uIndirSnrIdx,
                                        uint16_t uTriangType,
                                        bool_t bLargeObj,
                                        float32_t fX,
                                        float32_t fY,
                                        uint16_t uDirDist,
                                        uint16_t uIndirDist,
                                        uint16_t uBaseDist,
                                        uint16_t uSigStrength)
{
    uint8_t uConf;
    uint8_t uEchoCalcFlags;
    US_S_SnrPoint_t *sSnrNewPnt = &sSnrNewPntBuff[uNumNewPnts];
    const US_S_SnrCalcs_t * psSnrCalcs = UssMgr_psGetCurSnrCalcsRec();

    /* Calculate distance map information */
    sSnrNewPnt->uIndirDist = uIndirDist;
    if (uDirDist != ZERO)
    {
        sSnrNewPnt->uLowDirDist = uDirDist;
    }
    else
    {
        sSnrNewPnt->uLowDirDist = US_D_OD_DEFAULT_DIST;
    }

    /* Store last measuring sensors for min and mid distance calculations */
    sSnrNewPnt->uDirSnrIdx = uDirSnrIdx;
    sSnrNewPnt->uIndirSnrIdx = uIndirSnrIdx;

    /* Store initial point data */
    sSnrNewPnt->fX           = fX;
    sSnrNewPnt->fY           = fY;
    sSnrNewPnt->uAge         = ZERO;
    sSnrNewPnt->uCruisingAge = ZERO;

    uEchoCalcFlags = psSnrCalcs->sSnrCalcRec[uDirSnrIdx].sEchoCalc[uDirEchoIdx].uEchoCalcFlags;

    // Check if fake NFD.
    if (   ((uEchoCalcFlags & US_D_CALCFLAG_FAKE) != ZERO)
        && ((uEchoCalcFlags & US_D_CALCFLAG_HIGH_CONF) != ZERO))
    {
        sSnrNewPnt->uDecRate     = US_D_OD_INIT_FAKENFD_NOMEAS_DEC_RATE_EXP;
    }
    else
    {
        sSnrNewPnt->uDecRate     = US_D_OD_INIT_NOMEAS_DEC_RATE_EXP;
    }


    sSnrNewPnt->uBaseDist   =  uBaseDist;
#if (USS_FEATURE_ZONE_INFO == US_STD_ON)
    sSnrNewPnt->uSectorIdx   = UssOD_uFindSectorIdx(fX, fY, 0xFF);
#endif
    sSnrNewPnt->uOutputId    = US_D_OD_UNUSED_OUTPUT_IDX;
    sSnrNewPnt->fAccuX        = 0.0f;
    sSnrNewPnt->fAccuY        = 0.0f;

    /* Set height flag   Arbitration:  Once set as high point, stays high point. */
    if (bLargeObj == (bool_t)ME_TRUE)
    {
        sSnrNewPnt->uHeight = TWO;
    }
    else
    {
        sSnrNewPnt->uHeight = ONE;
    }

    // Check if fake NFD. If so, confidence forced at max.
    if (   ((uEchoCalcFlags & US_D_CALCFLAG_FAKE) != ZERO)
        && ((uEchoCalcFlags & US_D_CALCFLAG_HIGH_CONF) != ZERO))
    {
        // Assuming high conf, for now, for all fake nfd.
        //     if ((uEchoCalcFlags & US_D_CALCFLAG_HIGH_CONF) == ZERO)
        uConf = US_D_OD_PNT_MAX_CONF;
    }
    else
    {
        /* Set initial confidence */
        /* Special starting confidence for direct only points */
        if ((uTriangType & US_D_PNTSTAT_DIRECT_ONLY) == US_D_PNTSTAT_DIRECT_ONLY)
        {
            //use a tiny conf boost
            uConf = US_D_OD_INITAL_CONF_DIR_ONLY;
        }
        else
        /* Normal starting confidence logic */
        {
            if (uDirEchoIdx == ZERO) // Need better algorithm for closest point determination.
            {
                uConf = US_D_OD_INITAL_CONF_CLOSE;
            }
            else
            {
                uConf = US_D_OD_INITAL_CONF_STD;
            }
        }

        uConf = UssOD_CalcNewPtConf(uConf, uGrpIdx, uSigStrength, uDirDist, uTriangType);
    }

    /*
     * Confidence capping.  Cap initial conf if requested.  May be lifted later
     */

    /* Remove caching latch for current meas if point close to vehicle */
    if (uDirDist <= US_D_OD_CONF_CAP_LOCKOUT_RANGE)
    {
        uTriangType &= ~((uint16_t)US_D_PNTSTAT_CONF_CAPPED_LTCH); /* Remove caching latch for current meas*/
    }
    else{/*do nothing*/}

    /* Cap points that have conf capping set */
    if ((uTriangType & US_D_PNTSTAT_DIR_IND_TRIANG) != ZERO)
    {
        if (   ((uTriangType & US_D_PNTSTAT_CONF_CAPPED_LTCH) != ZERO)
            && (uConf > US_D_OD_MAYBE_CONF_CAP_DI))
        {
            uConf = US_D_OD_MAYBE_CONF_CAP_DI;
        }
    }
    else // Not DI measurement
    {
        if (   ((uTriangType & US_D_PNTSTAT_CONF_CAPPED_LTCH) != ZERO)
            && (uConf > US_D_OD_MAYBE_CONF_CAP_NOT_DI))
        {
            uConf = US_D_OD_MAYBE_CONF_CAP_NOT_DI;
        }
        else{/*do nothing*/}
    }

    /* Store confidence and increment new point count */
    sSnrNewPnt->uConf = uConf;

    /* Set status for diagnostic purposes */
    sSnrNewPnt->uPntStat = uTriangType | US_D_PNTSTAT_MEASURED; // We have been measured.

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
    sSnrNewPnt->uDeadPntStat = US_D_DPS_NULL;
    sSnrNewPnt->uMaxConf = uConf;
#endif

    /* Initialize sort index for this cycle */

    if (uNumNewPnts < US_D_POINT_BUFFER_SIZE)
    {
#if (US_D_OD_USE_FAST_SORT == ME_TRUE)
        /* Initialize sort index for this cycle */
        uSnrNewPntSortIdx[uNumNewPnts] = uNumNewPnts;
#endif
        uNumNewPnts++;
    }
    else{/*do nothing*/}

}
/*===========================================================================
 * @brief TBD
 *

 * @param  uint8_t uDirSnrIdx,
 * @param uint8_t uIndirSnrIdx,
 * @param uint8_t uDirEchoIdx,
 * @param uint16_t uSigStrength,
 * @param uint16_t uDirDist,
 * @param float32_t fPntAng,
 * @param float32_t fArea,      // For diag only - Add preprocessor define around
 * @param float32_t fTriHeight  // For diag only - Add preprocessor define around
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_ComputeFinalXYAndStore
 * Remarks:  DD-ID: {4A550339-A503-4c2a-9E93-5DB262077F08}
 * ===========================================================================*/
static void UssOD_ComputeFinalXYAndStore(uint8_t uGrpIdx,
                                         uint8_t uDirSnrIdx,
                                         uint8_t uIndirSnrIdx,
                                         uint8_t uDirEchoIdx,
                                         uint16_t uSigStrength,
                                         uint16_t uDirDist,
                                         uint16_t uIndirDist,
                                         uint16_t uBaseDist,
                                         float32_t fPntAng,
#if US_D_OD_DEBUG_TRIANG == ME_TRUE
                                         float32_t fArea,         // For diagnostics only.
                                         float32_t fTriHeight,    // For diagnostics only.
#endif
                                         bool_t  bLargeObj,
                                         uint16_t uTriangType)
{
    uint8_t uMatchPntIdx;
    float32_t fX;
    float32_t fY;

    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }
    else{/*do nothing*/}

    if (pSnrCoord == NULL)
    {
        return;   // Check if data exists.  Early return for functional safety.
    }
    else{/*do nothing*/}

    /*
     * Calculate point x,y from radius and angle,
     * into to autosar vehicle coordinate system.
     */
    fX = pSnrCoord[uDirSnrIdx].fX + (float32_t) uDirDist * (float32_t)cosf(fPntAng);
    fY = pSnrCoord[uDirSnrIdx].fY + (float32_t) uDirDist * (float32_t)sinf(fPntAng);

#ifdef US_D_OD_ENABLE_2D_OBJECTS
    /* Used for 2D Objects */
    if (uTriangType != US_D_PNTSTAT_DIRECT_ONLY || uDirDist < 75) // store if it is not a lone direct
    {
        sSingleSensorMeasList[nNumSingleSensorMeas].x = fX / 100.0f;
        sSingleSensorMeasList[nNumSingleSensorMeas].y = fY / 100.0f;
        //sSingleSensorMeasList[nNumSingleSensorMeas].uGrpIdx = uGrpIdx;
        sSingleSensorMeasList[nNumSingleSensorMeas].uDirSnrIdx = uDirSnrIdx;
        sSingleSensorMeasList[nNumSingleSensorMeas].uIndirSnrIdx = uIndirSnrIdx;
        sSingleSensorMeasList[nNumSingleSensorMeas].uDirEchoIdx = uDirEchoIdx;
        sSingleSensorMeasList[nNumSingleSensorMeas].uSigStrength = uSigStrength;
        //sSingleSensorMeasList[nNumSingleSensorMeas].uDirDist = uDirDist;
        //sSingleSensorMeasList[nNumSingleSensorMeas].uIndirDist = uIndirDist;
        //sSingleSensorMeasList[nNumSingleSensorMeas].fPntAng = fPntAng;
        sSingleSensorMeasList[nNumSingleSensorMeas].bLargeObj = bLargeObj;
        sSingleSensorMeasList[nNumSingleSensorMeas].uTriangType = uTriangType;
        //TBD: would be good to include the firing mode info.
        nNumSingleSensorMeas++;
    }
#endif /*US_D_OD_ENABLE_2D_OBJECTS*/

    /*
     * CHECK FOR MATCH WITH *EXISTING POINTS*
     */
    if (   (uTriangType == US_D_PNTSTAT_DIRECT_ONLY)
        || (uTriangType == US_D_PNTSTAT_DIR_DIR_TRING))
    {
        uMatchPntIdx = UssOD_MatchPointWithPntList(uGrpIdx, fX, fY, ME_FALSE);
    }
    else /* Bias matching ON for TD and DI */
    {
        uMatchPntIdx = UssOD_MatchPointWithPntList(uGrpIdx, fX, fY, ME_TRUE);
    }

    /* If there IS a match with the existing POINTS MAP! */
    if (uMatchPntIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND)
    {
        bool_t bMergePnts = (bool_t)(((uMatchPntIdx & US_D_OD_BIAS_MATCH_FLAG) == ZERO));
        uMatchPntIdx &= ~((uint8_t) US_D_OD_BIAS_MATCH_FLAG);

        UssOD_MergePntToGrpPntList(uGrpIdx,
                                   uMatchPntIdx,
                                   uDirSnrIdx,
                                   uIndirSnrIdx,
                                   uTriangType,
                                   bLargeObj,
                                   fX,
                                   fY,
                                   uDirDist,
                                   uIndirDist,
                                   uBaseDist,
                                   uSigStrength,
                                   bMergePnts);

    }
    else /* CHECK FOR MATCH WITH *NEW POINT LIST* */
    {
        uMatchPntIdx = UssOD_MatchPointWithNewList(uGrpIdx, fX, fY);

        /* We have a match with a point we just created, but not added to existing point map yet */
        if (uMatchPntIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND)
        {
            /* If there IS a match with the NEW POINTS MAP! */
            UssOD_MergePntToNewPntList(uGrpIdx,
                                       uMatchPntIdx,
                                       uDirSnrIdx,
                                       uIndirSnrIdx,
                                       uTriangType,
                                       bLargeObj,
                                       fX,
                                       fY,
                                       uDirDist,
                                       uIndirDist,
                                       uBaseDist,
                                       uSigStrength);
        }
        else /* NO MATCHES FOUND!  *CREATE NEW POINT* */
        {

#if US_D_OD_DEBUG_TRIANG == ME_TRUE
            /* DEBUG ONLY DATA */
            sSnrNewPntBuff[uNumNewPnts].fHeight      = fTriHeight;
            sSnrNewPntBuff[uNumNewPnts].fArea        = fArea;
            /* END DEBUG ONLY DATA */
#endif

            UssOD_CreatePntInNewPntList(uGrpIdx,
                                        uDirEchoIdx,
                                        uDirSnrIdx,
                                        uIndirSnrIdx,
                                        uTriangType,
                                        bLargeObj,
                                        fX,
                                        fY,
                                        uDirDist,
                                        uIndirDist,
                                        uBaseDist,
                                        uSigStrength);
        }
    }
}

/* ===========================================================================
 * Name:	 UssOD_CheckFOV
 * Remarks:  DD-ID: {C29AF5F6-33AA-49b5-B975-5F8D4494324B}
 * Req.-ID: 13514834
 * Req.-ID: 13515317
 * Req.-ID: 13515329
 * Req.-ID: 13515337
 * Req.-ID: 13515341
 * Req.-ID: 13515347
 * Req.-ID: 13515408
 * Req.-ID: 13515410
 * Req.-ID: 13515411
 * Req.-ID: 13515412
 * Req.-ID: 13515413
 * Req.-ID: 13515414
 * ===========================================================================*/
/*===========================================================================
 * @brief Check if FOV of point is invalidated by angular FOV
 *
 * @param float32_t fLeftPntAng
 * @param float32_t fRightPntAng
 * @param float32_t const US_S_SnrFOVCalc_t * psSnrFOVCalcInf
 * @return bool_t bFovCheck
 * @remarks
 */
static bool_t UssOD_CheckFOV(float32_t fLeftPntAng,
                             float32_t fRightPntAng,
                             const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo,
                             bool_t bCheckInnerFov)
{
    if (   (fLeftPntAng > psSnrFOVCalcInfo->fLeftFovMaxAng)
        || (fRightPntAng > psSnrFOVCalcInfo->fRightFovMaxAng))
    {
        return ME_FALSE;  // One or more FOV checks fail.
    }
    else if (((bool_t)ME_TRUE == bCheckInnerFov)
             &&
                (   (fRightPntAng < psSnrFOVCalcInfo->fRightFovMinAng)
                 || (fLeftPntAng < psSnrFOVCalcInfo->fLeftFovMinAng)))
    {
        return (bool_t)ME_FALSE;  // One or more FOV checks fail.
    }
    else
    {
        return (bool_t)ME_TRUE;   // All FOV checks pass
    }
}

#if US_D_OD_USE_SQUARE_FOV_CHECK == ME_TRUE
/*===========================================================================
 * @brief Check if FOV of point is invalidated by squared off FOV
 *
 * @param bool_t bFovCheck
 * @param bool_t bIsLeftIndir
 * @param float32_t fPntAng
 * @param float32_t uDirDist
 * @param float32_t uIndirDist
 * @param const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo
 * @return bool_t bFovCheck
 * @remarks
 */

static bool_t UssOD_CheckSquareFOV(bool_t bFovCheck,
                                   bool_t bIsLeftIndir,
                                   float32_t fPntAng,
                                   float32_t uDirDist,
                                   float32_t uIndirDist,
                                   const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo)
{
    float32_t fX;

    fX = (float32_t) uDirDist * (float32_t)cosf(fPntAng);  // Find X coord of point in Snr Pair Coord Sys

    if (ME_TRUE == bIsLeftIndir)
    {
        fX -= (psSnrFOVCalcInfo->fHalfSnrDist * 2.0f);  // If indirect, then adjust the point relative to the range of acceptable values.
    }

    if (uDirDist >= psSnrFOVCalcInfo->fSqFovDirMinDist)   // If direct echo in FOV range for rectangular limits.
    {
        if (fX < psSnrFOVCalcInfo->fSqFovDirX)  // Check far right rectangular FOV x value.
        {
            if (uIndirDist >= psSnrFOVCalcInfo->fSqFovIndirMinDist)
            {
                if (fX < psSnrFOVCalcInfo->fSqFovIndirX)
                {
                    bFovCheck = ME_FALSE;
                }
            }
        }
        else
        {
            bFovCheck = ME_FALSE;
        }
    }

    return bFovCheck;
}
#endif
/*===========================================================================
 * @brief Triangulate all the echoes between one direct and one indirect sensor.
 *
 * @param const US_S_SnrCalcs_t * psSnrCalcs
 * @param uint8_t uGrpIdx
 * @param uint8_t uDirSnrDataIdx
 * @param uint8_t uIndirSnrDataIdx
 * @param US_E_SnrDir_t eDirection
 * @return bool_t used for triang
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_TriangEchoesDirToIndir
 * Remarks:  DD-ID: {C399CF2D-2CAF-400b-BF96-6F7B6AE30BD2}
 * ===========================================================================*/
/*KPK-QAC Fix : Need to suppress, multiple branch/loop statements doesn't lead to any complexcity */
static bool_t UssOD_TriangEchoesDirToIndir(const US_S_SnrCalcs_t * psSnrCalcs,
                                           uint8_t uGrpIdx,
                                           uint8_t uDirSnrDataIdx,
                                           uint8_t uIndirSnrDataIdx,
                                           uint8_t uFovCalcIdx,
                                           bool_t  bIsLeftIndir)/* PRQA S 2755 */
{
    const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo = &(UssOD_psGetFOVCalcInfo()[uFovCalcIdx]);
    uint8_t   uDirEchoIdx;
    uint8_t   uIndirEchoIdx;
    bool_t    bLargeObj;
    bool_t    bFovCheck;
    bool_t    bFirstEchoUsedForTriang = ME_FALSE;
    bool_t    bIsNFD = ME_FALSE;
    uint16_t  uDirDist;
    uint16_t  uSigStrength;
    uint16_t  uIndirDist;
    uint16_t  uTotalDist;
    uint16_t  uBaseDist;
    uint32_t  uIndirDistSq;
    uint32_t  uDirDistSq;
    float32_t fArea;
    float32_t fAreaSq;
    float32_t fTriHeight;
    float32_t fPntAng;
    float32_t fIndirPntAng;
    float32_t fTmpCalc;
    float32_t fMinTotalDist;
    float32_t fMinAreaSq;

    /* Get number of direct and indirect echoes */
    uint8_t uNumDirEchoes = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].uNumEchoes;
    uint8_t uNumIndirEchoes = psSnrCalcs->sSnrCalcRec[uIndirSnrDataIdx].uNumEchoes;

    /* Limit to OD max echos to be considered */
    uNumDirEchoes = (uNumDirEchoes <= US_D_OD_MAX_ECHOS_FOR_TRIANG) ? uNumDirEchoes : US_D_OD_MAX_ECHOS_FOR_TRIANG;
    uNumIndirEchoes = (uNumIndirEchoes <= US_D_OD_MAX_ECHOS_FOR_TRIANG) ? uNumIndirEchoes : US_D_OD_MAX_ECHOS_FOR_TRIANG;

    /* Perform comparison between each direct echo with indirect echoes. */
    for (uDirEchoIdx = ZERO; uDirEchoIdx < uNumDirEchoes; uDirEchoIdx++)
    {
        /* Get direct echo distance and cross sectional acoustical strength approximation */
        uDirDist = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].uDirectDist_cm;
        uSigStrength = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].uSignalStrength;

        /* Is this a high object? */
        bLargeObj = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].bLargeObject;

#if US_D_OD_PATCH_PARALLEL_ECHOES_ONLY== ME_TRUE
        /* Loop through all parallel indexed direct and indirect echoes */
        if (uDirEchoIdx < uNumIndirEchoes)
        {
            uIndirEchoIdx = uDirEchoIdx; //set the indirect to use
#else
        /* Loop through all possible permutations of direct and indirect echoes */
        for (uIndirEchoIdx = ZERO; uIndirEchoIdx < uNumIndirEchoes; uIndirEchoIdx++)
        {
#endif
            if (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].eEchoType == US_E_NFD)
            {
                sObjDetCalcRec[uDirSnrDataIdx].bNearFieldSeen = ME_TRUE;
                bIsNFD = ME_TRUE;

                fMinTotalDist = psSnrFOVCalcInfo->fHalfSnrDist * 2.0f + 1.0f; // To ensure no signal transmission via bumper
                fMinAreaSq = 0.0f;
            }
            else
            {
                fMinTotalDist = psSnrFOVCalcInfo->fMinTotalDist;
                fMinAreaSq = psSnrFOVCalcInfo->fMinAreaSq;
            }

            /* Loop through the indirect sensor's echoes. */
            /* Get the total distance traveled by the sound from direct sensor to indirect sensor */
            uTotalDist = psSnrCalcs->sSnrCalcRec[uIndirSnrDataIdx].sEchoCalc[uIndirEchoIdx].uTotalDist_cm;

            /* Do initial - Super Cheap - low FOV test */
            if (   ((float32_t)uTotalDist >= fMinTotalDist)  // Check low FOV Angles
                && (uTotalDist >= uDirDist))                        // Check indirect TOF distance plausibility
            {
                /* Passed first test.  Calculated the distance to the object as measured by the indirect sensor */
                uIndirDist = uTotalDist - uDirDist;

                /* If distance is valid then proceed */
                /* May be a moot check, considering above (uTotalDist >= uDirDist)
                 *    AND (uTotalDist >= psSnrFOVCalcInfo[uFovCalcIdx].fMinTotalDist) */
                /* Might be able to reduce cyclomatic complexity by eliminating */
                if ((uDirDist != ZERO) && (uIndirDist != ZERO))
                {
                    /* Initial tests passed.  Attempt to triangulate point. */
                    fAreaSq = UssOD_CalcTriangAreaSq(psSnrFOVCalcInfo->fHalfSnrDist, uDirDist, uIndirDist);

                    /* Check if area squared passes FOV test */
                    if (fAreaSq > fMinAreaSq)
                    {
                        /* Now, finally, do computationally expensive math on plausible point */
                        fArea = (float32_t)sqrtf(fAreaSq);
                        fTriHeight = fArea / psSnrFOVCalcInfo->fHalfSnrDist;

                        /* Find inner angle of triangle at direct sensor */
                        fTmpCalc = fTriHeight / (float32_t) uDirDist;

                        /* Sometimes a float32_t rounding error can cause an issue. Deal with it. */
                        if (fTmpCalc <= 1.0f)
                        {
                            fPntAng = (float32_t)asinf(fTmpCalc);
                        }
                        else
                        {
                            fPntAng = (float32_t) M_PI_2;
                        }


                        /* Find inner angle of triangle at indirect sensor */
                        fTmpCalc = fTriHeight / (float32_t) uIndirDist;

                        /* Sometimes a float32_t rounding error can cause an issue. Deal with it. */
                        if (fTmpCalc <= 1.0f)
                        {
                            fIndirPntAng = (float32_t)asinf(fTmpCalc);
                        }
                        else
                        {
                            fIndirPntAng =(float32_t)  M_PI_2;
                        }

                        /* Check if obtuse triangle and adjust if necessary */
                        uDirDistSq = ((uint32_t)uDirDist * (uint32_t)uDirDist);
                        uIndirDistSq =((uint32_t)uIndirDist * (uint32_t)uIndirDist);

                        if ((float32_t) uIndirDistSq > ((float32_t) uDirDistSq + psSnrFOVCalcInfo->fSnrDistSq))
                        {
                            fPntAng =(float32_t) (M_PI - fPntAng);               // Adjust angle due to obtuse triangle.
                        }
                        else{/*do nothing*/}

                        if ((float32_t) uDirDistSq > ((float32_t) uIndirDistSq + psSnrFOVCalcInfo->fSnrDistSq))
                        {
                            fIndirPntAng =(float32_t) (M_PI - fIndirPntAng);    // Adjust angle due to obtuse triangle.
                        }
                        else{/*do nothing*/}

                        /* If we are using the left side indirect, mirror the triangle
                         * Angular FOV check.
                         */
                        if (bIsLeftIndir == (bool_t)ME_TRUE)
                        {
                            fPntAng =(float32_t)( M_PI - fPntAng);  // Mirror the triangle to the left of the direct sensor.
                            bFovCheck = (bool_t)(UssOD_CheckFOV(fIndirPntAng, fPntAng, psSnrFOVCalcInfo, (bool_t) (ME_FALSE == bIsNFD)));
                        }
                        else
                        {
                            fIndirPntAng =(float32_t)( M_PI - fIndirPntAng);  // Mirror the indirect to the right of the direct sensor.
                            bFovCheck = (bool_t)(UssOD_CheckFOV(fPntAng, fIndirPntAng, psSnrFOVCalcInfo, (bool_t) (ME_FALSE == bIsNFD)));

                        }

#if US_D_OD_USE_SQUARE_FOV_CHECK == ME_TRUE
                        /* Rectangular, final FOV check. */
                        if (bFovCheck == ME_TRUE)
                        {
                            bFovCheck = UssOD_CheckSquareFOV(bFovCheck,     // If current FOV check is affirmative, then limit further to rectangular FOV.
                                                             bIsLeftIndir,
                                                             fPntAng,
                                                             (float32_t) uDirDist,
                                                             (float32_t) uIndirDist,
                                                             psSnrFOVCalcInfo);
                        }
#endif

                        /* If all FOV checks pass, then store it! */
                        if (bFovCheck == (bool_t)ME_TRUE)
                        {
                            uint16_t uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
                            /* Release conf camp when performing flanking mitigation to
                             * allow objects in mitigation that have been triangulated
                             * in a single measurement, to grow in conf.
                             */
                            bool_t bIsFlankingMitigationActive = UssOD_bIsFlankingMitigationActive();
                            if ((bool_t) ME_TRUE == bIsFlankingMitigationActive)
                            {
                                uMaybeCapConf = ZERO;  /* Shut off conf capping in this scenario. */
                            }
#endif
                            /* FOV test passed.  We have a plausible point. */
                            /* Mark cached direct echo as used for triangulation */
                            if (ZERO == uDirEchoIdx)
                            {
                                bFirstEchoUsedForTriang = ME_TRUE;
                            }
                            else{/*do nothing*/}

                            /* Rotate coordinate system from sensor pair to vehicle coordinate system */
                            fPntAng += psSnrFOVCalcInfo->fSnrPairAng;
                            float32_t fBaseDist = (psSnrFOVCalcInfo->fHalfSnrDist * 2.0f); /* Using an intermediate var is required for QAC compliance.  Lame. */
                            uBaseDist = (uint16_t) (fBaseDist);

                            /* Time to store the point in the right place. */
                            UssOD_ComputeFinalXYAndStore(uGrpIdx,          // Group Index
                                                         uDirSnrDataIdx,   // Direct sensor index
                                                         uIndirSnrDataIdx, // Indirect sensor index
                                                         uDirEchoIdx,      // Direct sensor echo index
                                                         uSigStrength,     // Signal strength
                                                         uDirDist,         // Direct distance
                                                         uIndirDist,
                                                         uBaseDist,
                                                         fPntAng,          // Direct point angle
    #if US_D_OD_DEBUG_TRIANG == ME_TRUE
                                                         fArea,            // DIAG ONLY: Area of triangle
                                                         fTriHeight,       // DIAG ONLY: Triangle height
    #endif
                                                         bLargeObj,
                                                         (US_D_PNTSTAT_DIR_IND_TRIANG | US_D_PNTSTAT_DIR_IND_TRIANG_LTCH | uMaybeCapConf));
                        }
                    }
                }
            }
        }
    }

    return bFirstEchoUsedForTriang;
}

/*===========================================================================
 * @brief Triangulate all the echoes between one direct and one direct sensor.
 *
 * @param const US_S_SnrCalcs_t * psSnrCalcs
 * @param uint8_t uGrpIdx
 * @param uint8_t uDirSnrDataIdx
 * @param uint8_t uIndirSnrDataIdx
 * @param US_E_SnrDir_t eDirection
 * @return bool_t Used for triangulation.
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_TriangEchoesDirToDir
 * Remarks:  DD-ID: {3A21CCE9-A19D-4ab1-B0D8-0695469C3886}
 * Req.-ID: 13514834
 * ===========================================================================*/
static bool_t UssOD_TriangEchoesDirToDir(const US_S_SnrCalcs_t * psSnrCalcs,
                                         uint8_t uGrpIdx,
                                         uint8_t uDirSnrDataIdx,
                                         uint8_t uIndirSnrDataIdx,
                                         uint8_t uFovCalcIdx,
                                         bool_t  bIsLeftIndir)
{
    const US_S_SnrFOVCalc_t * psSnrFOVCalcInfo = &(UssOD_psGetFOVCalcInfo()[uFovCalcIdx]);
    uint8_t   uDirEchoIdx;
    bool_t    bLargeObj;
    bool_t    bFovCheck;
    bool_t    bFirstEchoUsedForTriang = ME_FALSE;
    bool_t    bIsNFD = ME_FALSE;
    uint16_t  uDirDist;
    uint16_t  uSigStrength;
    uint16_t  uIndirDist = ZERO;
    uint16_t  uBaseDist;
    uint16_t  uTotalDist = ZERO;
    uint32_t  uIndirDistSq;
    uint32_t  uDirDistSq;
    float32_t fArea;
    float32_t fAreaSq;
    float32_t fTriHeight;
    float32_t fPntAng;
    float32_t fIndirPntAng;
    float32_t fTmpCalc;
    float32_t fVirtSnrPairAng;  // Angle of virtual sensor pair in vehicle coordinate system.
    float32_t fAdjSnrPairAng;   // Adjusted angle of sensor pair in vehicle coordinate system, with respect to orig pair
    float32_t fAdjPntAng;       // Direct point angle adjusted from virtual pair coord system to pair coord system
    float32_t fDeltaX;
    float32_t fDeltaY;
    float32_t fVirtSnrDist;
    float32_t fVirtHalfSnrDist;
    float32_t fMinTotalDist;
    float32_t fMinAreaSq;

    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;
    US_S_DirEchoCache_t *psDirEchoCache = &sObjDetCalcRec[uIndirSnrDataIdx].sDirEchoCache;
    const US_S_EchoCacheEntry_t *psEchoCacheEntry = &psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV];

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }
    else{/*do nothing*/}

    if(pSnrCoord == NULL)
    {
        return (bool_t)ME_FALSE;   // Check if data exists.  Early return for functional safety.
    }
    else{/*do nothing*/}

    /* Get number of direct and indirect echoes */
    uint8_t uNumDirEchoes = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].uNumEchoes;

    /* Limit to OD max echos to be considered */
    uNumDirEchoes = (uNumDirEchoes <= US_D_OD_MAX_ECHOS_FOR_TRIANG) ? uNumDirEchoes : US_D_OD_MAX_ECHOS_FOR_TRIANG;

    /* Perform comparison between each direct echo with indirect echoes. */
    if (uNumDirEchoes >= ONE)
    {
        uDirEchoIdx = ZERO;  // Only using echo zero for caching at this time.

        /* Get direct echo distance and cross sectional acoustical strength approximation */
        uDirDist = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].uDirectDist_cm;
        uSigStrength = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].uSignalStrength;

        /* Get the direct echo distance from the current calculation record.
         * Get the indirect echo distance from L0 (Prev) cache.
         * Use this distance plus the current direct distance to triangulate
         * Mostly the same algo can is used that is used for direct/indirect triangulation.
         */

        if ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
        {
            // Load distance from cache
            uIndirDist = psEchoCacheEntry->uDirDist;
            uTotalDist =  (uint16_t)((uIndirDist + uDirDist) & 0xFFFFu);
        }
        else{/*do nothing*/}

        if ((uDirDist != ZERO) && (uIndirDist != ZERO))
        {
            /* Calculate virtual pair angle, being careful to always subtract the
             * right sensor minus the left sensor on the bumper to get correct pair angle.
             */

            if (bIsLeftIndir == (bool_t)ME_TRUE)
            {
                // Indirect on left, direct on right
                fDeltaX = pSnrCoord[uDirSnrDataIdx].fX - psEchoCacheEntry->fVirtualSnrPosX;
                fDeltaY = pSnrCoord[uDirSnrDataIdx].fY - psEchoCacheEntry->fVirtualSnrPosY;
            }
            else
            {
                // Indirect is on Right, direct on left
                fDeltaX = psEchoCacheEntry->fVirtualSnrPosX - pSnrCoord[uDirSnrDataIdx].fX;
                fDeltaY = psEchoCacheEntry->fVirtualSnrPosY - pSnrCoord[uDirSnrDataIdx].fY;
            }

            if (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].eEchoType == US_E_NFD)
            {
                sObjDetCalcRec[uDirSnrDataIdx].bNearFieldSeen = ME_TRUE;
                bIsNFD = ME_TRUE;

                fMinTotalDist = 10.0f; /*psSnrFOVCalcInfo->fMinTotalDist*/
                fMinAreaSq = 0.0f;
            }
            else
            {
                fMinTotalDist = 10.0f; /*psSnrFOVCalcInfo->fMinTotalDist*/
                fMinAreaSq = 50.0f; /*psSnrFOVCalcInfo->fMinAreaSq*/
            }

            /* Calculate distance between sensors */
            fVirtSnrDist = (float32_t)sqrtf(fDeltaX * fDeltaX + fDeltaY * fDeltaY); // Calculate distance between sensors.
            fVirtHalfSnrDist = fVirtSnrDist * 0.5f;

            /* Do initial - Super Cheap - low FOV test */
            if (    ((float32_t) uTotalDist >= fMinTotalDist)  // Check low FOV Angles
                 && (uTotalDist >= uDirDist)                         // Check indirect TOF distance plausibility
                 && (fVirtSnrDist > 10.0f))                      // Insure plausible distance between sensors
            {
                /* Initial tests passed.  Attempt to triangulate point. */
                fAreaSq = UssOD_CalcTriangAreaSq(fVirtHalfSnrDist, uDirDist, uIndirDist);

                /* Check if area squared passes FOV test */
                if (fAreaSq > fMinAreaSq) // > 10 cm equilateral triangle
                {
                    /* Now, finally, do computationally expensive math on plausible point */
                    fArea = (float32_t)sqrtf(fAreaSq);
                    fTriHeight = fArea / fVirtHalfSnrDist;

                    /* Find inner angle of triangle at direct sensor */
                    fTmpCalc = fTriHeight / (float32_t) uDirDist;

                    /* Sometimes a float32_t rounding error can cause an issue. Deal with it. */
                    if (fTmpCalc <= 1.0f)
                    {
                        fPntAng = (float32_t)asinf(fTmpCalc);
                    }
                    else
                    {
                        fPntAng = (float32_t) M_PI_2;
                    }


                    /* Find inner angle of triangle at indirect sensor */
                    fTmpCalc = fTriHeight / (float32_t) uIndirDist;

                    /* Sometimes a float32_t rounding error can cause an issue. Deal with it. */
                    if (fTmpCalc <= 1.0f)
                    {
                        fIndirPntAng = (float32_t)asinf(fTmpCalc);
                    }
                    else
                    {
                        fIndirPntAng =  (float32_t) M_PI_2;
                    }

                    /* Check if obtuse triangle and adjust if necessary */
                    uDirDistSq =((uint32_t)uDirDist * (uint32_t)uDirDist);
                    uIndirDistSq = ((uint32_t)uIndirDist * (uint32_t)uIndirDist);

                    if ((float32_t) uIndirDistSq > ((float32_t) uDirDistSq + fVirtSnrDist * fVirtSnrDist))
                    {
                        fPntAng = (float32_t)(M_PI - fPntAng);               // Adjust direct angle due to obtuse triangle.
                    }

                    if ((float32_t) uDirDistSq > ((float32_t) uIndirDistSq + fVirtSnrDist * fVirtSnrDist))
                    {
                        fIndirPntAng =(float32_t)(M_PI - fIndirPntAng);    // Adjust indirect angle due to obtuse triangle.
                    }

                    /* When using directed odometry, must adjust point angle to deal with time delay */

                    /* Slope angle of sensor pair, as viewed from leftmost sensor */
                    fVirtSnrPairAng = (float32_t)atan2f(fDeltaY, fDeltaX);  // Store for later coordinate transformations

                    /*
                     * +fAdjSnrPairAng: PCS -> VPCS
                     * -fAdjSnrPairAng:  VPCS -> PCS
                     */

                    // Create translation angle for PCS/VPCS
                    fAdjSnrPairAng = fVirtSnrPairAng - psSnrFOVCalcInfo->fSnrPairAng;

                    /* Check if virtual sensor pair angle has same left-right orientation
                     * as the regular sensor pair angle.  Pass: -pi/2 < fAdjSnrPairAng <= pi/2
                     */
                    if (fAdjSnrPairAng >= 0.0f)
                    {
                        bFovCheck =(bool_t)(fAdjSnrPairAng < M_PI_2);
                    }
                    else
                    {
                        bFovCheck = (bool_t)(fAdjSnrPairAng > -M_PI_2);
                    }

                    /* If we are using the left side indirect, mirror the triangle,
                     * And do final FOV check.
                     */
                    if (bFovCheck == (bool_t)ME_TRUE)
                    {
                        if (bIsLeftIndir == (bool_t)ME_TRUE)
                        {
                            fPntAng = (float32_t)(M_PI - fPntAng);  // Mirror the triangle to the left of the direct sensor.
                            fAdjPntAng = fPntAng - fAdjSnrPairAng;  // Translate the point angle in VPCS into PCS.

                            fIndirPntAng -= fAdjSnrPairAng;  // Translate the indirect point angle in VPCS into PCS.

                            bFovCheck = (bool_t)UssOD_CheckFOV(fIndirPntAng, fAdjPntAng, psSnrFOVCalcInfo,(bool_t)( ME_FALSE == bIsNFD));
                        }
                        else
                        {
                            fIndirPntAng =(float32_t)(M_PI - fIndirPntAng);  // Mirror the indirect to the right of the direct sensor.
                            fIndirPntAng -= fAdjSnrPairAng;      // Translate the indirect point angle in VPCS into PCS.

                            fAdjPntAng = fPntAng - fAdjSnrPairAng; // Translate the point angle in VPCS into PCS.

                            bFovCheck = (bool_t)UssOD_CheckFOV(fAdjPntAng, fIndirPntAng, psSnrFOVCalcInfo, (bool_t)( ME_FALSE == bIsNFD));
                        }
                    }

#if US_D_OD_USE_SQUARE_FOV_CHECK == ME_TRUE
                    /* Rectangular, final FOV check. */
                    if (bFovCheck == ME_TRUE)
                    {
                        bFovCheck = UssOD_CheckSquareFOV(bFovCheck,     // If current FOV check is affirmative, then limit further to rectangular FOV.
                                                         bIsLeftIndir,
                                                         fPntAng,
                                                         (float32_t) uDirDist,
                                                         (float32_t) uIndirDist,
                                                         psSnrFOVCalcInfo);
                    }
#endif

                    if (bFovCheck == (bool_t)ME_TRUE)
                    {
                        /* FOV test passed.  We have a plausible point.
                         * Mark indirect cached echo AND current direct measurement,
                         * as used for triangulation, in calling function.
                         */
                        if (ZERO == uDirEchoIdx)
                        {
                            bFirstEchoUsedForTriang = ME_TRUE; // The current measurement was triangulated.  Inform calling function.
                        }
                        else{/*do nothing*/}

                        /* Did this direct echo have points merged? If so, it's probably large or high. */
                        bLargeObj = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].bLargeObject;

                        /* Rotate coordinate system from sensor pair to vehicle coordinate system */
                        //fPntAng += psSnrFOVCalcInfo->fSnrPairAng;
                        fPntAng += fVirtSnrPairAng;

                        uBaseDist = (uint16_t) fVirtSnrDist;
                        /* Time to store the point in the right place. */
                        UssOD_ComputeFinalXYAndStore(uGrpIdx,          // Group Index
                                                     uDirSnrDataIdx,   // Direct sensor index
                                                     uIndirSnrDataIdx, // Indirect sensor index
                                                     uDirEchoIdx,      // Direct sensor echo index
                                                     uSigStrength,     // Signal strength
                                                     uDirDist,         // Direct distance
                                                     uIndirDist,
                                                     uBaseDist,
                                                     fPntAng,          // Direct point angle
#if US_D_OD_DEBUG_TRIANG == ME_TRUE
                                                     fArea,            // DIAG ONLY: Area of triangle
                                                     fTriHeight,       // DIAG ONLY: Triangle height
#endif
                                                     bLargeObj,
                                                     (US_D_PNTSTAT_DIR_DIR_TRING | US_D_PNTSTAT_DIR_DIR_TRING_LTCH));         // Use virtual sensor position
                    }  // End of FOV Check
                } // End of squared area check
            } // End of plausibility check
        } // End of dir and indir dist check.
    } // End of num of echoes check.

    return bFirstEchoUsedForTriang;
}

#define US_D_CACHE_LOOP_UNDERFLOW (0xFFu)
/*===========================================================================
 * @brief Triangulate all the echoes between one sensor and itself via motion
 *
 * @param const US_S_SnrCalcs_t * psSnrCalcs
 * @param uint8_t uGrpIdx
 * @param uint8_t uDirSnrDataIdx
 * @param uint8_t uIndirSnrDataIdx
 * @param US_E_SnrDir_t eDirection
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:     UssOD_TriangEchoesDirToDir
 * Remarks:  DD-ID: {3A21CCE9-A19D-4ab1-B0D8-0695469C3886}
 * Req.-ID: 13514834
 * ===========================================================================*/
static bool_t UssOD_TriangEchoesTemporalDir(const US_S_SnrCalcs_t * psSnrCalcs,
                                            uint8_t uGrpIdx,
                                            uint8_t uDirSnrDataIdx)
{
    uint8_t   uDirEchoIdx;
    uint8_t   uIndirSnrDataIdx;
    uint8_t   uMatchPntIdx;
    uint8_t   uCacheIdx;
    uint8_t   uCacheActiveBit;
    bool_t    bFovCheck;
    bool_t    bLargeObj = ME_FALSE;
    bool_t    bFinishTriang = ME_TRUE;
    bool_t    bIsLeftIndir;
    bool_t    bWasUsedForTriang = ME_FALSE;
    bool_t    bMaybeMakeNewAltPnt;
    bool_t    bSuppressDefaultPnt;
    bool_t    bIsSideSensor;
    bool_t    bIsInnerSensor;
    bool_t    bIsRearGroup;
    bool_t    bIsFapaActive = ME_FALSE;
    bool_t    bMergePnts;
    uint16_t  uMaybeCapConf = ZERO;
    uint16_t  uDirDist;
    uint16_t  uSigStrength;
    uint16_t  uIndirDist;
    uint16_t  uBaseDist;
    uint16_t  uTotalDist;
    uint16_t  uDeltaDist;
    uint32_t  uIndirDistSq;
    uint32_t  uDirDistSq;
    float32_t fArea;
    float32_t fAreaSq;
    float32_t fTriHeight;
    float32_t fPntAng;
    float32_t fIndirPntAng;
    float32_t fTmpCalc;
    float32_t fVirtSnrPairAng;  // Angle of virtual sensor pair in vehicle coordinate system.
    float32_t fSnrAng;
    float32_t fDeltaX;
    float32_t fDeltaY;
    float32_t fVirtSnrDist;
    float32_t fVirtHalfSnrDist;
    float32_t fAltPntAng;
    float32_t fDefX;
    float32_t fDefY;
    float32_t fAltX;
    float32_t fAltY;
    float32_t fMinBaseDist;
    float32_t fAngFromNormal;
    float32_t fVehSpeed_kph = 0.0f;

    US_S_DirEchoCache_t *psDirEchoCache;
    US_S_EchoCacheEntry_t *psEchoCacheEntry;
    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;
    const US_S_SnrAngleCalcs_t * psSnrAngleCalcs  = UssOD_psGetSnrAngleCalcs();   /* Get angle calculations */
    US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();

    fSnrAng = psSnrAngleCalcs->fSnrAngInVehCoords_rad[uDirSnrDataIdx];

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }
    else{/*do nothing*/}

    if(pSnrCoord == NULL)
    {
        return (bool_t)ME_FALSE;   // Check if data exists.  Early return for functional safety.
    }
    else{/*do nothing*/}

    uIndirSnrDataIdx = uDirSnrDataIdx;

    /* Get number of direct and indirect echoes */
    uint8_t uNumDirEchoes = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].uNumEchoes;

    /* Limit to OD max echos to be considered */
    uNumDirEchoes = (uNumDirEchoes <= US_D_OD_MAX_ECHOS_FOR_TRIANG) ? uNumDirEchoes : US_D_OD_MAX_ECHOS_FOR_TRIANG;

    /* Perform comparison between each direct echo with indirect echoes. */

    #if (US_D_TD_UNIT_TESTING_MODE != 0)
        uNumDirEchoes = 1u;
    #endif
    if (uNumDirEchoes >= ONE)
    {
        uDirEchoIdx = ZERO; // Cache only stores echo zero at this time.

        if (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].eEchoType == US_E_NFD)
        {
            sObjDetCalcRec[uDirSnrDataIdx].bNearFieldSeen = ME_TRUE;
        }
        else{/*do nothing*/}

        bIsRearGroup = (bool_t)(pSnrCoord[uDirSnrDataIdx].fX <= 0.0f); // Are we on the left side of the vehicle?

        bIsSideSensor =  (bool_t)((uDirSnrDataIdx == US_D_SENSOR_FSR)
                       || (uDirSnrDataIdx == US_D_SENSOR_FSL)
                       || (uDirSnrDataIdx == US_D_SENSOR_RSR)
                       || (uDirSnrDataIdx == US_D_SENSOR_RSL));

        /* Get direct echo distance and cross sectional acoustical strength approximation */
        uDirDist = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].uDirectDist_cm;
        uSigStrength = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].uSignalStrength;

        /* ODO is moving at too small a rate to properly match with distances with 1 cm res.
         * Shut down temporal in this scanario.
         */

#if (US_D_USE_FAPA_API == ME_TRUE)
        TbAP_DriveAssistStatOut_t sDriveAssistStatOut;
        SigMgr_PpDriveAssistStatOut_TbAP_DriveAssistStatOut_t_Get(&sDriveAssistStatOut);
        /* Do not perform mitigation in FAPA mode. */
        bIsFapaActive = (TeAutoPark_e_AutoParkStatus_Active == sDriveAssistStatOut.IeAP_e_AutoParkStatus)
                        && (TeAP_e_AlgoState_Park_In == sDriveAssistStatOut.IeAP_e_AlgoState);
#endif
#if (OD_D_USE_FAPA_FEATURE_WITH_NO_API == ME_TRUE)
        bIsFapaActive = ME_TRUE;
#endif
        /*
         * If vehicle is moving very slowly odo will be changing at such a small rate that the angular change at this
         * distance is significant.  It is better to skip temporal triangulation and let the lone directs measure these
         * close distances.
         */
        if (   ((bool_t)ME_TRUE == bIsFapaActive)
            && ((bool_t)ME_FALSE == bIsSideSensor))
        {
            fVehSpeed_kph = UssCom_F_GetVehSpeed();
            if (   (uDirDist < 50u)
                && (fVehSpeed_kph < 2.0f)
                && (fVehSpeed_kph > 0.0f))
            {
                bFinishTriang = (bool_t) ME_FALSE;
            }
            else{/*do nothing*/}
        }
        else{/*do nothing*/}

        if ((bool_t)ME_TRUE == bFinishTriang)
        {
            bFinishTriang =(bool_t) ME_FALSE;

            /* Get the direct echo distance from the direct echo calculations.
             * Use this distance plus the current direct distance to form an virtual
             * indirect sensor return . Rhe same algo can be used that is used for
             * direct/indirect triangulation.
             */
            psDirEchoCache = &sObjDetCalcRec[uDirSnrDataIdx].sDirEchoCache;

            /* Loop through cache entries */
            uCacheActiveBit = US_D_ECHO_CACHE_L2_ACTIVE;

            // Loop through caches in L3->L1 order
            for (uCacheIdx = US_D_NUM_ECHO_CACHE_ENTRIES; uCacheIdx-- > ZERO; )
            {
                /* Adjust cached fixed distance if active */
                if ((psDirEchoCache->uEchoCacheUseStat & uCacheActiveBit) != ZERO)
                {
                    // Retrieve the echo cache entry for this sensor and cache depth.
                    psEchoCacheEntry = &psDirEchoCache->sEchoCacheEntry[uCacheIdx];

                    if (psDirEchoCache->sEchoCacheEntry[uCacheIdx].uDirDist == ZERO)
                    {
                        continue;
                    }
                    else{/*do nothing*/}

                    fDeltaX      = psEchoCacheEntry->fDeltaX;
                    fDeltaY      = psEchoCacheEntry->fDeltaY;
                    fVirtSnrDist = psEchoCacheEntry->fPairDist;

                    // Insure plausible distance between sensors. 5 lateral movement expected for above 0.863 kph)
                    if (fVirtSnrDist < US_D_CACHE_MIN_SNR_BASE_DIST)
                    {
                        continue; // Don't want to use this cache.  Too small or too big virtual pair base dist.
                    }
                    else{/*do nothing*/}

                    // We made it this far.  Cache entry still ok.
                    // Next try triang.

                    // 0 = No Test
    #if (US_D_TD_UNIT_TESTING_MODE != ZERO)
                    uSigStrength = 200u;
                    UssOD_TemporalDirUnitTest(uDirSnrDataIdx,
                                              bIsRearGroup,
                                              &uDirDist,
                                              &uIndirDist,
                                              fDeltaX,
                                              pSnrCoord,
                                              psEchoCacheEntry);
    #else
                    uIndirDist = psDirEchoCache->sEchoCacheEntry[uCacheIdx].uDirDist;
    #endif

                    uTotalDist =  (uint16_t)((uIndirDist + uDirDist) & 0xFFFFu);

                    /* Automatic odometry distance tolerance correction */
                    if (uIndirDist > uDirDist)
                    {
                        fMinBaseDist = (float32_t)uIndirDist -  (float32_t)uDirDist;
                    }
                    else
                    {
                        fMinBaseDist = (float32_t)uDirDist -  (float32_t)uIndirDist;
                    }

                    if (fVirtSnrDist < fMinBaseDist)
                    {
                        // How man centimeters are we short of 180 degree triangle.
                        if ((fMinBaseDist - fVirtSnrDist) <= 2.0f)  // Correct up to 2 cm of error.
                        {
                            fVirtSnrDist = fMinBaseDist;
                        }
                        else
                        {
                            continue; // This triangulation isn't going to work.
                        }
                    }
                    else{/*do nothing*/}

                    if ((uDirDist != ZERO) && (uIndirDist != ZERO) && (uTotalDist >= uDirDist))
                    {
                        /* Initial tests passed.  Attempt to triangulate point. */

                        fVirtHalfSnrDist = fVirtSnrDist * 0.5f;
                        uBaseDist = (uint16_t) fVirtSnrDist;

                        fAreaSq = UssOD_CalcTriangAreaSq(fVirtHalfSnrDist, uDirDist, uIndirDist);

                        /* Check if area squared passes FOV test */
                        if (fAreaSq >= 0.0f)
                        {
                            bFinishTriang = (bool_t) ME_TRUE;
                            break;
                        }
                        else{/*do nothing*/}
                    }
                    else{/*do nothing*/}
                }
                uCacheActiveBit >>= ONE; // Check the next Cache Active
            }
        }

#if 0 // Nothing to do.
          // Loop failed to find any good virtual sensor pairs.
        if (uCacheIdx == US_D_CACHE_LOOP_UNDERFLOW)
        {
            // No good triangulation found with any of cache. bFinishTriang is FALSE by default.
            // Fall out of function and do nothing more.
        }
#endif

        if ((bool_t)ME_TRUE == bFinishTriang)
        {
            /*
             * Determine triangulation direction
             * First determine if we are on left side of vehicle.
             * If so, direction of travel will determine triangulation direction.
             */
            bIsLeftIndir = (bool_t) (pSnrCoord[uDirSnrDataIdx].fY >= 0.0f); // Are we on the left side of the vehicle?

            /* End debug stuff */
            if (fDeltaX > 0.0f) // Reverse
            {
                if ((bool_t)ME_TRUE == bIsLeftIndir) // Sensor is to the LEFT of current sensor position
                {
                    bIsLeftIndir = (bool_t)ME_FALSE; // If reversing, on left side of the vehicle, triangulate right-to-left
                }
                else
                {
                    bIsLeftIndir = (bool_t)ME_TRUE; // If reversing, on right side of the vehicle, triangulate left-to-right
                }
            }
            else{/*do nothing*/}

            /* Now, finally, do computationally expensive math on plausible point */
            fArea = (float32_t)sqrtf(fAreaSq);
            fTriHeight = fArea / fVirtHalfSnrDist;

            /* Find inner angle of triangle at direct sensor */
            fTmpCalc = fTriHeight / (float32_t) uDirDist;

            /* Sometimes a float32_t rounding error can cause an issue. Deal with it. */
            if (fTmpCalc <= 1.0f)
            {
                fPntAng = (float32_t)asinf(fTmpCalc);
            }
            else
            {
                fPntAng = (float32_t)M_PI_2;
            }

            /* Find inner angle of triangle at indirect sensor */
            fTmpCalc = fTriHeight / (float32_t) uIndirDist;

            /* Sometimes a float32_t rounding error can cause an issue. Deal with it. */
            if (fTmpCalc <= 1.0f)
            {
                fIndirPntAng = (float32_t)asinf(fTmpCalc);
            }
            else
            {
                fIndirPntAng = M_PI_2;
            }

            /* Check if obtuse triangle and adjust if necessary */
            uDirDistSq =((uint32_t)uDirDist * (uint32_t)uDirDist);
            uIndirDistSq =((uint32_t)uIndirDist * (uint32_t)uIndirDist);

            if ((float32_t) uIndirDistSq > ((float32_t) uDirDistSq + fVirtSnrDist * fVirtSnrDist))
            {
                fPntAng =(float32_t)( M_PI - fPntAng);               // Adjust direct angle due to obtuse triangle.
            }
            else{/*do nothing*/}

            if ((float32_t) uDirDistSq > ((float32_t) uIndirDistSq + fVirtSnrDist * fVirtSnrDist))
            {
                fIndirPntAng = (float32_t)(M_PI - fIndirPntAng);    // Adjust indirect angle due to obtuse triangle.
            }
            else{/*do nothing*/}

            /* Slope angle of sensor pair, as viewed from leftmost sensor */
            fVirtSnrPairAng = (float32_t)atan2f(fDeltaY, fDeltaX);  // Store for later coordinate transformations

            /* Rotate coordinate system from sensor pair to vehicle coordinate system */
            if (bIsLeftIndir == (bool_t)ME_TRUE)
            {
                fAltPntAng = fVirtSnrPairAng + fPntAng;
                fPntAng    = fVirtSnrPairAng - fPntAng;
            }
            else
            {
                fAltPntAng = fVirtSnrPairAng - fPntAng;
                fPntAng    = fVirtSnrPairAng + fPntAng;
            }
            /* Calculation for point on alternate side of pair */
            /* Did this direct echo have points merged? If so, it's probably large or high. */
            bLargeObj = psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[uDirEchoIdx].bLargeObject;

            /*****************************************************************************************
             * ALT POINT angle (reflected around sensor pair axis, towards interior of the vehicle)
             *****************************************************************************************
             */
            bSuppressDefaultPnt = ME_FALSE;

            if ((bool_t)ME_TRUE == bIsSideSensor)
            {
                bMaybeMakeNewAltPnt = (bool_t)ME_FALSE; // No not make alt point for side sensors

                /* Doesn't get in here anyway.  Side exculdued at top. */
#if (ME_FALSE == US_D_OD_USE_TEMPORAL_FOR_SIDE_SNRS)
                if ((bool_t)ME_TRUE == bIsFapaActive)
                {
                    if (uIndirDist > uDirDist)
                    {
                        uDeltaDist = uIndirDist - uDirDist;
                    }
                    else
                    {
                        uDeltaDist = uDirDist - uIndirDist;
                    }

                    if (uDeltaDist < 3u) /* 3 cm */
                    {
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)                        
                        dbg_FapaSideTDDisable = 1u;
#endif                        
                        bSuppressDefaultPnt = (bool_t)ME_TRUE;  /* Suppress making default point when passing along a flat surface on side */
                    }
                }
                else
                {
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)                    
                    dbg_FapaSideTDDisable = 0u;
#endif                
                }
#else
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
                dbg_FapaSideTDDisable = 2u;
#endif
#endif
            }
            else
            {
                /* Make alt point possible */
                fAltX = pSnrCoord[uDirSnrDataIdx].fX + (float32_t) uDirDist * (float32_t)cosf(fAltPntAng);
                fAltY = pSnrCoord[uDirSnrDataIdx].fY + (float32_t) uDirDist * (float32_t)sinf(fAltPntAng);

                bMaybeMakeNewAltPnt = ME_TRUE;
            }

            fDefX = pSnrCoord[uDirSnrDataIdx].fX + (float32_t) uDirDist * (float32_t)cosf(fPntAng);
            fDefY = pSnrCoord[uDirSnrDataIdx].fY + (float32_t) uDirDist * (float32_t)sinf(fPntAng);

            if ((bool_t)ME_TRUE == bMaybeMakeNewAltPnt)
            {
                uMatchPntIdx = US_D_OD_NO_MATCH_FOUND;

                // If alt point is inside vehicle tube, maybe we can match it.
                if ((fAltY <= psDrivingTubeInfo->fTubeEdgeY) && (fAltY >= -psDrivingTubeInfo->fTubeEdgeY))
                {
                    /* Check for match point before making new point */
                    uMatchPntIdx = UssOD_MatchPointWithPntList(uGrpIdx, fAltX, fAltY, ME_TRUE);
                }
                else{/*do nothing*/}

                /* Alt Points from the Y axis of the sensor, inwards towards inside
                 * of vehicle can only be matched with exting points, otherwise
                 * the point will be created from the Y axis of the sensor outwards.
                 */
                if (uMatchPntIdx != (uint8_t) US_D_OD_NO_MATCH_FOUND)  /* match found. */
                {
                    bIsInnerSensor =  (bool_t)(   (uDirSnrDataIdx == US_D_SENSOR_FIR)
                                               || (uDirSnrDataIdx == US_D_SENSOR_FIL)
                                               || (uDirSnrDataIdx == US_D_SENSOR_RIR)
                                               || (uDirSnrDataIdx == US_D_SENSOR_RIL));

                    if ((bool_t)ME_TRUE == bIsInnerSensor)
                    {
                        if ((bool_t)ME_TRUE == bIsRearGroup) // rear
                        {
                            if (fAltX > psDrivingTubeInfo->fRearBumperX)
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }
                            else{/*do nothing*/}
                        }
                        else // front
                        {
                            if (fAltX < psDrivingTubeInfo->fFrontBumperX)
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }
                            else{/*do nothing*/}
                        }
                    }

                    /* Note: If statements could be combined after debugging is complete */

                    /* If inner sensor */
                    /* Is point angle within fov or direct sensor */
                    fAngFromNormal = fAltPntAng - fSnrAng;

                    if (fAngFromNormal <= 0.0f)
                    {
                        fAngFromNormal =-fAngFromNormal;
                    }
                    else{/*do nothing*/}

                    if (fAngFromNormal >= M_2PI)
                    {
                        fAngFromNormal -= M_2PI;
                    }
                    else{/*do nothing*/}

                    if (fAngFromNormal > SNR_PDC_FOV_2_RAD)  // Selection needs to come from cals.  Hard-coded for now.
                    {
                        uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                        uSigStrength = ZERO;
                    }
                    else{/*do nothing*/}

                    if ((ME_TRUE == bIsInnerSensor) && (ZERO == uMaybeCapConf))
                    {
                        bool_t bInTubeBounds =(bool_t) (   ((fAltY < psDrivingTubeInfo->fTubeEdgeY + 10.0f))
                                                        && ((fAltY > -psDrivingTubeInfo->fTubeEdgeY - 10.0f)));

                        switch (uDirSnrDataIdx)
                        {
                            case US_D_SENSOR_FIR:
                            case US_D_SENSOR_FIL:
                            {
                                psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_FIL].sDirEchoCache;
                                if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                    && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                    && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                                & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                                {
                                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                    uSigStrength = ZERO;
                                }

                                psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_FIR].sDirEchoCache;
                                if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                    && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                    && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                                & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                                {
                                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                    uSigStrength = ZERO;
                                }

                                /* Even though we have passed fov, diminish conf (confcap)  points that are close to bumper. */
                                if (   (fAltX < (psDrivingTubeInfo->fFrontBumperX + 80.0f))
                                    && (ME_TRUE == bInTubeBounds))
                                {
                                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                    uSigStrength = ZERO;
                                }

                                break;

                            }

                            case US_D_SENSOR_RIR:
                            case US_D_SENSOR_RIL:
                            {
                                psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_RIL].sDirEchoCache;
                                if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                    && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                    && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                                & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                                {

                                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                    uSigStrength = ZERO;
                                }


                                psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_RIR].sDirEchoCache;
                                if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                    && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                    && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                                & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                                {
                                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                    uSigStrength = ZERO;
                                }

                                /* Even though we have passed fov, diminish conf (confcap)  points that are close to bumper. */
                                if (   (fAltX < (psDrivingTubeInfo->fRearBumperX - 80.0f))
                                    && ((bool_t)ME_TRUE == bInTubeBounds))
                                {
                                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                    uSigStrength = ZERO;
                                }

                                break;
                            }
                            default:
                            {
                                 /* We don't want to take any action.  Leave the confcap and sigstrength alone. */
                            }
                              break;
                        }
                    }

                    /* Only allow matching default point with point inside tube if default point also in tube */
                    if ((fDefY <= psDrivingTubeInfo->fTubeEdgeY) && (fDefY >= -psDrivingTubeInfo->fTubeEdgeY))
                    {
                        bSuppressDefaultPnt = ME_TRUE;  // Suppress making default point

                        /* Flag current direct for temporal cached measurement */
                        bWasUsedForTriang = ME_TRUE;

                        bMergePnts =(bool_t)((uMatchPntIdx & US_D_OD_BIAS_MATCH_FLAG) == ZERO);
                        uMatchPntIdx &= (uint8_t)~( US_D_OD_BIAS_MATCH_FLAG);

                        /* Annotate group point with the matched temportal lone direct measurement */
                        UssOD_MergePntToGrpPntList(uGrpIdx,
                                                   uMatchPntIdx,
                                                   uDirSnrDataIdx,
                                                   uIndirSnrDataIdx,            // No indirect sensor.
                                                   (US_D_PNTSTAT_TDIR_DIR_TRIANG | US_D_PNTSTAT_TDIR_DIR_TRIANG_LTCH | uMaybeCapConf),
                                                   bLargeObj,
                                                   fAltX,
                                                   fAltY,
                                                   uDirDist,
                                                   uIndirDist,
                                                   uBaseDist,
                                                   uSigStrength,
                                                   bMergePnts);
                    }
                }
            }


            /*****************************************************************************************
             * DEFAULT POINT angle (towards exterior of the vehicle)
             *****************************************************************************************
             */

            if ((bool_t)ME_FALSE == bSuppressDefaultPnt)
            {
                /*
                 * Confidence capping FOV checks.
                 */

                bFovCheck = ME_TRUE;
                uMaybeCapConf = ZERO;

                if ((bool_t)ME_TRUE == UssOD_bIsPntInsideVehicle(fDefX, fDefY))
                {
                    bFovCheck = ME_FALSE;  // Invalidate temporal triangulation for this direct.
                }


                bIsInnerSensor = (bool_t)(   (uDirSnrDataIdx == US_D_SENSOR_FIR)
                                          || (uDirSnrDataIdx == US_D_SENSOR_FIL)
                                          || (uDirSnrDataIdx == US_D_SENSOR_RIR)
                                          || (uDirSnrDataIdx == US_D_SENSOR_RIL));

                /* Inner sensor code may never get here due to new condition at top of function */
                if ((bool_t)ME_TRUE == bIsInnerSensor)
                {
                    if ((bool_t)ME_TRUE == bIsRearGroup) // rear
                    {
                        if (fDefX > psDrivingTubeInfo->fRearBumperX)
                        {
                            uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                            uSigStrength = ZERO;
                        }
                    }
                    else // front
                    {
                        if (fDefX < psDrivingTubeInfo->fFrontBumperX)
                        {
                            uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                            uSigStrength = ZERO;
                        }
                    }
                }

                /* Note: If statements could be combined after debugging is complete */

                /* If inner sensor */
                /* Is point angle within fov or direct sensor */
                fAngFromNormal = fPntAng - fSnrAng;

                if (fAngFromNormal <= 0.0f)
                {
                    fAngFromNormal =-fAngFromNormal;
                }

                if (fAngFromNormal >= M_2PI)
                {
                    fAngFromNormal -= M_2PI;
                }

                if (fAngFromNormal > SNR_PDC_FOV_2_RAD)  // Selection needs to come from cals.  Hard-coded for now.
                {
                    uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                    uSigStrength = ZERO;
                }

                if ((ME_TRUE == bIsInnerSensor) && (ZERO == uMaybeCapConf))
                {
                    bool_t bInTubeBounds =    (bool_t)(   ((fDefY < psDrivingTubeInfo->fTubeEdgeY + 10.0f))
                                                       && ((fDefY > -psDrivingTubeInfo->fTubeEdgeY - 10.0f)));

                    switch (uDirSnrDataIdx)
                    {
                        case US_D_SENSOR_FIR:
                        case US_D_SENSOR_FIL:
                        {
                            psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_FIL].sDirEchoCache;
                            if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                            & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }

                            psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_FIR].sDirEchoCache;
                            if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                            & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }

                            /* Even though we have passed fov, diminish conf (confcap)  points that are close to bumper. */
                            if (   (fDefX < (psDrivingTubeInfo->fFrontBumperX + 80.0f))
                                && (ME_TRUE == bInTubeBounds))
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }

                            break;

                        }

                        case US_D_SENSOR_RIR:
                        case US_D_SENSOR_RIL:
                        {
                            psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_RIL].sDirEchoCache;
                            if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                            & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                            {

                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }


                            psDirEchoCache = &sObjDetCalcRec[US_D_SENSOR_RIR].sDirEchoCache;
                            if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                                && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO)
                                && ((psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uMeasUseStat
                                            & (US_D_ECHO_USAGE_STAT_USED_DI_TRIANG | US_D_ECHO_USAGE_STAT_USED_DD_TRIANG)) != ZERO))
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }

                            /* Even though we have passed fov, diminish conf (confcap)  points that are close to bumper. */
                            if (   (fDefX < (psDrivingTubeInfo->fRearBumperX - 80.0f))
                                && (ME_TRUE == bInTubeBounds))
                            {
                                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;
                                uSigStrength = ZERO;
                            }

                            break;
                        }
                        default:
                        {
                               /* We don't want to take any action.  Leave the confcap and sigstrength alone. */
                            break;
                        }
                    }
                }

                /* Check for match point before making new point */
                uMatchPntIdx = UssOD_MatchPointWithPntList(uGrpIdx, fDefX, fDefY, ME_TRUE);
                bMergePnts = (bool_t)((uMatchPntIdx & US_D_OD_BIAS_MATCH_FLAG) == ZERO);

#if (ME_TRUE == US_D_OD_USE_RADIAL_MATCHING_FOR_TD)
                /* If no match found, try searching again using radial match */
                if (uMatchPntIdx == US_D_OD_NO_MATCH_FOUND)
                {
                    uMatchPntIdx = UssOD_MatchMeasDistWithPntList(uGrpIdx,
                                                                  uDirSnrDataIdx,
                                                                  uDirDist);
                    bMergePnts = ME_FALSE;
                }
#endif
                if (uMatchPntIdx == (uint8_t)US_D_OD_NO_MATCH_FOUND)
                {
                    /*
                     * Hard FOV check for side sensors.  Failure results in no temporal point created.
                     */

                    if (ME_TRUE == bIsSideSensor)
                    {
                        if (   (ME_FALSE == bIsRearGroup)
                            && (fDefX < psDrivingTubeInfo->fFrontSnrClsX))
                        {
                            bFovCheck = ME_FALSE;  // Invalidate temporal triangulation for this direct.
                        }

                        if (   (ME_TRUE == bIsRearGroup)
                            && (fDefX > psDrivingTubeInfo->fRearSnrClsX))
                        {
                            bFovCheck = ME_FALSE;  // Invalidate temporal triangulation for this direct.
                        }
                    }

                    // If all fov checks pass
                    if (ME_TRUE == bFovCheck)
                    {
                        /* Flag current direct for temporal cached measurement */
                        bWasUsedForTriang = ME_TRUE;

                        /* Time to store the point in the right place. */
                        UssOD_ComputeFinalXYAndStore(uGrpIdx,          // Group Index
                                                     uDirSnrDataIdx,   // Direct sensor index
                                                     uIndirSnrDataIdx, // Indirect sensor index
                                                     uDirEchoIdx,      // Direct sensor echo index
                                                     uSigStrength,     // Signal strength
                                                     uDirDist,         // Direct distance
                                                     uIndirDist,
                                                     uBaseDist,
                                                     fPntAng,       // Direct point angle
            #if US_D_OD_DEBUG_TRIANG == ME_TRUE
                                                     fArea,            // DIAG ONLY: Area of triangle
                                                     fTriHeight,       // DIAG ONLY: Triangle height
            #endif
                                                     bLargeObj,
                                                     (US_D_PNTSTAT_TDIR_DIR_TRIANG | US_D_PNTSTAT_TDIR_DIR_TRIANG_LTCH | uMaybeCapConf));
                    }
                }
                else /* Default point matched with existing point. Merge or reinforce point. */
                {
                    /* If match was a side bias match, which only exists in (x,y) matching,
                     * bMergePnts will be true when NOT a bias match, thus will not alter
                     * the (x,y) of the original point when a bias match.
                     */

                    uMatchPntIdx &= (uint8_t)~( US_D_OD_BIAS_MATCH_FLAG);

                    /* Flag current direct for temporal cached measurement */
                    bWasUsedForTriang = ME_TRUE;

                    /* Annotate group point with the matched measurement */
                    /* fX and fY are not used when bMergePnts == ME_FALSE */
                    /* If there IS a match with the existing point map, then update point. */
                    UssOD_MergePntToGrpPntList(uGrpIdx,
                                               uMatchPntIdx,
                                               uDirSnrDataIdx,
                                               uIndirSnrDataIdx,            // No indirect sensor.
                                               (US_D_PNTSTAT_TDIR_DIR_TRIANG | US_D_PNTSTAT_TDIR_DIR_TRIANG_LTCH | uMaybeCapConf),
                                               bLargeObj,
                                               fDefX,
                                               fDefY,
                                               uDirDist,
                                               uIndirDist,
                                               uBaseDist,
                                               uSigStrength,
                                               bMergePnts);
                }
            }
        } // End (ME_TRUE == bFinishTriang)
    } // End if (uNumDirEchoes >= ONE)

    return bWasUsedForTriang;
}

/*===========================================================================
 * @brief Consider lone direct echos for point creation
 *
 * @param const US_S_SnrCalcs_t * psSnrCalcs
 * @param uint8_t uGrpIdx
 * @param uint8_t uDirSnrDataIdx
 * @param uint8_t uIndirSnrDataIdx
 * @param US_E_SnrDir_t eDirection
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_ConsiderLoneDirects
 * Remarks:  DD-ID: {D8B966A5-70D7-4f8c-8A54-B4E97BA7C365}
 * Req.-ID: 15001645
 * Req.-ID: 15001705
 * ===========================================================================*/
/*KPK-QAC Fix : Need to suppress, multiple branch/loop statements doesn't lead to any complexity */
static void UssOD_ConsiderLoneDirects(const US_S_SnrCalcs_t * psSnrCalcs,
                                      uint8_t uGrpIdx,
                                      uint8_t uSnrDataIdx)
{
#if (US_D_OD_USE_LONE_DIR_PNT_MATCHING == ME_TRUE)
    bool_t    bIsSideFacingSensor;
    uint8_t   uMatchPntIdx = US_D_OD_NO_MATCH_FOUND;
#endif
    uint8_t   uInnerSnrIdx;
    bool_t    bCreateNewPnt = ME_TRUE;
    bool_t    bLargeObj = ZERO;;
    bool_t    bIsNFD = ME_FALSE;
    uint16_t  uMaybeCapConf = ZERO;
    uint16_t  uDirDist = ZERO;
    uint16_t  uSigStrength;
    float32_t fPntAng;

    const US_S_SnrAngleCalcs_t * psSnrAngleCalcs  = UssOD_psGetSnrAngleCalcs();   /* Get angle calculations */
    /* Get number of direct echoes */
    uint8_t uNumDirEchoes = psSnrCalcs->sSnrCalcRec[uSnrDataIdx].uNumEchoes;

    /* Limit to OD max echos to be considered */
    uNumDirEchoes = (uNumDirEchoes <= US_D_OD_MAX_ECHOS_FOR_TRIANG) ? uNumDirEchoes : US_D_OD_MAX_ECHOS_FOR_TRIANG;

    if (psSnrCalcs->sSnrCalcRec[uSnrDataIdx].eEchoType == US_E_NFD) /*|| (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[ZERO].uDirectDist_cm < 40)*/
    {
        sObjDetCalcRec[uSnrDataIdx].bNearFieldSeen = ME_TRUE;
        bIsNFD = ME_TRUE;
    }


    psSnrAngleCalcs = UssOD_psGetSnrAngleCalcs();

    if (uNumDirEchoes >= ONE)
    {
        uDirDist = psSnrCalcs->sSnrCalcRec[uSnrDataIdx].sEchoCalc[ZERO].uDirectDist_cm;
        uSigStrength = psSnrCalcs->sSnrCalcRec[uSnrDataIdx].sEchoCalc[ZERO].uSignalStrength;
        bLargeObj = psSnrCalcs->sSnrCalcRec[uSnrDataIdx].sEchoCalc[ZERO].bLargeObject;


        /* Default Lone Direct Angle:  Use the pre-computed Theta Angle to place point in center of FOV */
        fPntAng = psSnrAngleCalcs->fSnrAngInVehCoords_rad[uSnrDataIdx]; // Angle of sensor in vehicle coord system.
    }


    if (uDirDist > ZERO)
    {
        /* Last check before creating lone direct.
         * Check if lone direct distance matches with previously created point.
         * Only need to check sSnrPnt list, as there will be no new points that will match
         * since two sensors on this firing cycle should be able to see same object,
         * due to cross-talk prevention.
         *
         * If match found, existing point will be annotated and lone direct will not be created.
         */
        // Do not perform point matching if this is a side facing sensor.

        if ((bool_t)ME_TRUE == bIsNFD) /*|| (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[ZERO].uDirectDist_cm < 40)*/
        {
            sObjDetCalcRec[uSnrDataIdx].bNearFieldSeen = ME_TRUE;
        }


#if ((US_D_OD_USE_LDPM_ONLY_REAR == ME_TRUE) || (US_D_OD_USE_LDPM_DISABLE_IN_NOISY == ME_TRUE))
        if (
    #if (US_D_OD_USE_LDPM_ONLY_REAR == ME_TRUE)
               (uGrpIdx == US_D_PHYS_GRP_REAR)
    #endif
    #if (US_D_OD_USE_LDPM_DISABLE_IN_NOISY == ME_TRUE)
            && (UssMgr_eGetGrpState(uGrpIdx) == SYSMGR_GRPSTATE_OK)
    #endif
           )
#endif
        {


#if (US_D_OD_USE_LONE_DIR_PNT_MATCHING == ME_TRUE)
            bCreateNewPnt = ME_TRUE;

            // Do not perform point matching if this is a side facing sensor.
            bIsSideFacingSensor =      (bool_t)((uSnrDataIdx == US_D_SENSOR_FSR)
                                    || (uSnrDataIdx == US_D_SENSOR_FSL)
                                    || (uSnrDataIdx == US_D_SENSOR_RSR)
                                    || (uSnrDataIdx == US_D_SENSOR_RSL));

            if (   (bIsSideFacingSensor == (bool_t)ME_FALSE)                   // This is NOT a side sensor
                && (uDirDist <= US_D_OD_LONE_POINT_MATCH_DIST_LIM))    // and within range to allow point matching.
            {
                /* Perform point matching check, and if radial match found. */
                uMatchPntIdx = UssOD_MatchMeasDistWithPntList(uGrpIdx,
                                                              uSnrDataIdx,
                                                              uDirDist);

                /* Do not create lone at sensor angle if match found */
                if (uMatchPntIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND)
                {
                    bCreateNewPnt = ME_FALSE;
                }
            }
#endif /* (US_D_OD_USE_LONE_DIR_PNT_MATCHING == ME_TRUE) */

#if (US_D_OD_WEAKEN_LONES_AT_DIST == ME_TRUE)
            /* Highly diminish confidence by capping conf */
            if (uDirDist >= US_D_OD_WEAKEN_DIST_FOR_LONES)
            {
                uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH; // Weaken all lone points at a distance of greater than 2m.
            }
#endif

#if (US_D_OD_DEL_LONES_WHEN_NO_INNER_MEAS == ME_TRUE)

            /*
             * If no match found then make determination to create a lone or not depending
             * on if any measurement is observed at associated inner sensor.
             * Only applies to NON-NFD measurements.
             */
            if (((bool_t)ME_TRUE == bCreateNewPnt) && ((bool_t)ME_FALSE == bIsNFD))
            {
                uint8_t uCheckListIdx;
                US_S_DirEchoCache_t *psDirEchoCache;

                for (uCheckListIdx = ZERO; uCheckListIdx < (uint8_t)US_D_OD_LONE_SNR_CHECK_MAX; uCheckListIdx++)
                {
                    if (uSnrDataIdx == sLoneDirSnrCheckList[uCheckListIdx][US_D_OD_LONE_SNR_CHECK_OUTER])
                    {
                        uInnerSnrIdx = sLoneDirSnrCheckList[uCheckListIdx][US_D_OD_LONE_SNR_CHECK_INNER];

                        psDirEchoCache = &sObjDetCalcRec[uInnerSnrIdx].sDirEchoCache;
                        if (   ((psDirEchoCache->uEchoCacheUseStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)
                            && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].uDirDist != ZERO))
                        {
                            uMaybeCapConf = US_D_PNTSTAT_CONF_CAPPED_LTCH;  // Cap confidence at a very low level.
                        }

                        break;
                    }
                }
            }
#endif
        }

		if ((bool_t)ME_TRUE == bCreateNewPnt)
        {
			UssOD_ComputeFinalXYAndStore(uGrpIdx,          // Group Index
										 uSnrDataIdx,   // Direct sensor index
										 0xFF,             // NO Indirect sensor index
										 ZERO,             // Direct sensor echo index
										 uSigStrength,     // Signal strength
										 uDirDist,         // Direct distance
										 ZERO,             // Indirect distance is zero.
										 ZERO,             // Base Distance
										 fPntAng,          // Direct point angle
	#if US_D_OD_DEBUG_TRIANG == ME_TRUE
										 0.0f,             // DIAG ONLY: Area of triangle = ZERO
										 0.0f,             // DIAG ONLY: Triangle height  = ZERO
	#endif
										 bLargeObj,        // Is this a large/high object?
										 (US_D_PNTSTAT_DIRECT_ONLY | US_D_PNTSTAT_DIRECT_ONLY_LTCH | uMaybeCapConf));         // Use virtual sensor position


		}
		else if (uMatchPntIdx != (uint8_t)US_D_OD_NO_MATCH_FOUND)
	    {
		    /* Annotate group point with the matched lone direct measurement */
	        /* fX and fY are not used when bMergePnts == ME_FALSE */
	        /* If there IS a match with the existing point map, then update point. */
	        UssOD_MergePntToGrpPntList(uGrpIdx,
	                                   uMatchPntIdx,
	                                   uSnrDataIdx,
	                                   0xFF,            // No indirect sensor.
	                                   (US_D_PNTSTAT_DIRECT_ONLY | US_D_PNTSTAT_DIRECT_ONLY_LTCH),
	                                   bLargeObj,
	                                   0.0f, // Merge points has to be set to FALSE or this will cause and issue!!
	                                   0.0f, // Merge points has to be set to FALSE or this will cause and issue!!
	                                   uDirDist,
	                                   ZERO,
	                                   ZERO,
	                                   uSigStrength,
	                                   ME_FALSE);
	    }
#if 0
		else // Very special case
		{
		    // Don't create point so that hopefully it will be resolved in next measurement.
		}
#endif
    }
}


/*===========================================================================
 * @brief Decrease confidence of each point in a physical group point map.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_AdjustConfLevelofPnts
 * Remarks:  DD-ID: {8ABE71ED-7854-4d12-96AB-7FE1476BCA9B}
 * Traceability to source Code: Req.-ID: 17275272,
 * Req.-ID: 17272021
 * ===========================================================================*/

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
/* Dead point resurrection debug data */
uint8_t _dbg_NecPnt_RTail = 0;
uint8_t _dbg_NecPnt_RHead = 0;
uint8_t _dbg_NecPnt_LTail = 0;
uint8_t _dbg_NecPnt_LHead = 0;
uint8_t _dbg_NecPnt_FapaMode = 0;
uint8_t _dbg_NecPnt_nNewPnt = 0;
#endif

static void UssOD_AgeAndAdjustOldPoints(uint8_t uGrpIdx, US_E_Grp_Allow_Level_t eGrpAllowLvl)
{
    uint8_t uListIdx;
    uint8_t uPntIdx;
    uint8_t uSnrIdx;
    uint8_t uConfDec;
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    uint8_t uDirSnrGrpIdx;
    uint8_t uNumSnrsInGrp;
    uint8_t uCacheIdx;
#if (ME_TRUE == US_D_OD_KEEP_DEAD_PTS)
    bool_t  bIsSideSensor;
    bool_t  bIsFapaActive = ME_FALSE;
#endif
#if (ME_TRUE == US_D_OD_VERY_SLOW_DEC_FOR_CLOSE_PNTS)
    bool_t bIsNoisy = ME_FALSE;
#endif
    uint16_t uCacheActiveBit;
    float32_t fX;
    float32_t fY;
    US_S_SnrPoint_t * psSnrPnt;
#if US_D_OD_USE_DIRECT_DIST_ODO == ME_TRUE
    US_S_DirEchoCache_t *psDirEchoCache;
    US_S_EchoCacheEntry_t *psEchoCacheEntry;
#endif
    const US_S_PlatPhysInfo_t * psPlatPhysInfo = UssCtrl_psGetPlatPhysInfo();
    const US_S_SnrCalcs_t * psSnrCalcs = UssMgr_psGetCurSnrCalcsRec();  // Get echo class calculations

#if (ME_TRUE == US_D_OD_VERY_SLOW_DEC_FOR_CLOSE_PNTS)
    bIsNoisy = (bool_t) (UssMgr_eGetGrpState(uGrpIdx) != SYSMGR_GRPSTATE_OK);  /* this covers NOISY and DEGRADED */
#endif

    UssOdo_OdoAccumOutput_t * psOdoAccumOut;
    float32_t fCos;
    float32_t fSin;
    bool_t bUseOdometry = US_D_OD_USE_ODOMETRY_FLAG_FUNC;

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
#if (US_D_USE_FAPA_API == ME_TRUE)
    TbAP_DriveAssistStatOut_t sDriveAssistStatOut;
    SigMgr_PpDriveAssistStatOut_TbAP_DriveAssistStatOut_t_Get(&sDriveAssistStatOut);
    /* Do not perform mitigation in FAPA mode. */
    bIsFapaActive = (TeAutoPark_e_AutoParkStatus_Active == sDriveAssistStatOut.IeAP_e_AutoParkStatus)
                    && (TeAP_e_AlgoState_Park_In == sDriveAssistStatOut.IeAP_e_AlgoState);
#endif
#endif

    /* Note:
     * Controller is a higher priority task and may modify psOdoAccumOut
     * while it is being used by this function.
     * A semaphore or mutex may be needed, but error would be very small if
     * it were to happen.   (error <= 4.722 cm)
     */
    /*KPK-QAC Fix : Need to suppress, 	intentional implementation  */
    if (bUseOdometry == (bool_t)ME_TRUE) /* PRQA S 2991,2995 *///function call
    {
        psOdoAccumOut = UssOdo_psGetOdoAccumOut();
        fCos = (float32_t)cosf(-psOdoAccumOut->fAngleDelta);
        fSin = (float32_t)sinf(-psOdoAccumOut->fAngleDelta);

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
        UssOD_UpdateOdometryForDeadPnts(uGrpIdx, fCos, fSin, psOdoAccumOut->fXdelta, psOdoAccumOut->fYdelta);
#endif

    }

    /*
     ************* FIRST POINT ANOTATION LOOP ************
     *  Loop through all points in THIS group
     */
    for (uListIdx = 0; uListIdx < uNumCurPnts; uListIdx++)
    {
        uPntIdx = puGrpSnrPntSortIdx[uListIdx]; // Get the sorted point index
        psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];  // Get the record for this sorted point

        /* Clear this group of points when this flag is set */
        if (eGrpAllowLvl == OD_GRP_CLR_N_SUPPRESS)
        {
            psSnrPnt->uConf = ZERO;
        }
        else if (eGrpAllowLvl == OD_GRP_REDUCE_CONF)
        {
            /* Lower confidence this group of points when this flag is set */
            if (psSnrPnt->uConf > US_OD_CONF_CAP_FOR_GROUP)
            {
                psSnrPnt->uConf = US_OD_CONF_CAP_FOR_GROUP;
            }
        }

        /* Only perform adjustments on points that are active.
         * If confidence of point is above zero, proceed
         */
        if (psSnrPnt->uConf > ZERO)
        {
            // Get the sensor who formed this point's index
            uSnrIdx = psSnrPnt->uDirSnrIdx;

            /* Increase age, limit to range, and store */
            if (psSnrPnt->uAge < US_D_OD_PNT_MAX_AGE)
            {
                psSnrPnt->uAge++;
            }

            /* Increase cruising age, limit to range, and store */
            if (psSnrPnt->uCruisingAge < US_D_OD_PNT_MAX_CRUISING_AGE)
            {
                psSnrPnt->uCruisingAge++;
            }

            /*
             * Shift point map due to Ego vehicle movement.
             * Apply Odometry adjustments.
             */
            /*KPK-QAC Fix : Need to suppress, 	intentional implementation  */
            if (bUseOdometry == (bool_t)ME_TRUE) /* PRQA S 2991,2995 */ //function call
            {
                /* Odometry */

                fX = psSnrPnt->fX;
                fY = psSnrPnt->fY;

                
                {
                    psSnrPnt->fX = fX * fCos - fY * fSin;
                    psSnrPnt->fY = fX * fSin + fY * fCos;
                }

                psSnrPnt->fX -= psOdoAccumOut->fXdelta;
                psSnrPnt->fY -= psOdoAccumOut->fYdelta;

#if (US_D_OD_USE_CR_DIST_KILL == ME_TRUE)
                /* Adjust distance kill algo tracking point */
                psSnrPnt->fAccuX = psSnrPnt->fAccuX + (psSnrPnt->fX - fX); //assumes fX holds prev position
                psSnrPnt->fAccuY = psSnrPnt->fAccuY + (psSnrPnt->fY - fY);
#endif

#if (USS_FEATURE_ZONE_INFO == US_STD_ON)
                /* Check for zone / sector updates */
                if (psSnrPnt->uCruisingAge > ONE)
                {
                    psMatchedPoint->uSectorIdx = UssOD_uFindSectorIdx(fX, fY, psMatchedPoint->uSectorIdx);
                }
#endif

                /* Keep Dead Points (KDP)
                 * Delete any kept point that is too far away in X.  Cheap and dirty.
                 */
#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)

                /* Dead point resurrection algo.
                 * Resurrect at best conf it ever had, so keep track of it.
                 */
                if (psSnrPnt->uConf > psSnrPnt->uMaxConf)
                {
                    psSnrPnt->uMaxConf = psSnrPnt->uConf;
                }

    #if (US_D_OD_KDP_DEL_PT_ON_DIST == ME_TRUE)
                /* Clear out kept points that are too far from vehicle in X.
                 * Quick and dirty X only check
                 */
                if ((psSnrPnt->uDeadPntStat & US_D_DPS_KEPT) != ZERO) /* flagged as kept */
                {
                    /* lazy way, only using X */
                    if (   (psSnrPnt->fX > US_D_OD_KDP_DEL_DIST_CM_FRNT)
                        || (psSnrPnt->fX < US_D_OD_KDP_DEL_DIST_CM_REAR)
                        || (ME_FALSE == bIsFapaActive))
                    {
                        psSnrPnt->uDeadPntStat |= US_D_DPS_DIST_DEL; /* mark it for debug */
                        psSnrPnt->uConf = ZERO; /* Delete the point completely */
                    }
                }
    #endif
#endif

            } /* End of Odometry Adjustments */

 /*
  *  Old Keep Dead Point (JR Necro Code)
  */
#if 0  // old code - kept for now - clean later
            /* Keep Dead Points (KDP) acceptance criteria */
            bIsSideSensor =   (uSnrIdx == US_D_SENSOR_FSR)
                           || (uSnrIdx == US_D_SENSOR_FSL)
                           || (uSnrIdx == US_D_SENSOR_RSR)
                           || (uSnrIdx == US_D_SENSOR_RSL);

            /* Check if point should be marked as accepted for keeping dead points */
            if  ( (psSnrPnt->uConf >= US_D_OD_KDP_ACCEPT_CONF)
    #if (US_D_OD_KDP_REJECT_LONES == ME_TRUE)
                && ((psSnrPnt->uPntStat & US_D_PNTSTAT_DIRECT_ONLY) == ZERO)
    #endif
    #if (US_D_OD_KDP_CHECK_SIDES_ONLY == ME_TRUE)

                && (ME_TRUE == bIsSideSensor)
    #endif
    #if (US_D_OD_KDP_LIMIT_ACCEPT_RANGE == ME_TRUE)
                && (US_D_OD_KDP_ACCEPT_RANGE_MAX_CM >= psSnrPnt->uLowDirDist)
                && (100 <= psSnrPnt->uLowDirDist)
    #endif
        )
            {
                psSnrPnt->uDeadPntStat |= US_D_DPS_KEPT;
            }
            /* Check if point is zombie but then gets remeasured later */
            if ( ((psSnrPnt->uDeadPntStat & US_D_DPS_KEPT) != ZERO)
                && (psSnrPnt->uConf > US_D_OD_KDP_DEAD_PT_CONF) )
            {
                psSnrPnt->uDeadPntStat = US_D_DPS_REMEAS; /* set to re-measured, clear other flags */
                psSnrPnt->uDeadPntStat |= US_D_DPS_KEPT; /* re-accept it */
            }
#endif

#if 0  // old code - kept for now - clean later
#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
            if (  (((psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoClass == US_E_DIRECT)
                    &&  (psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoType != US_E_NFD))
                    || (UssOD_puGetCycSinceLastMeas()[uSnrIdx] > US_D_MAX_NUM_SENSORS))
                    && ((psSnrPnt->uDeadPntStat & US_D_DPS_KEPT) == ZERO) ) /* not kept */
#else
            if (   ((psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoClass == US_E_DIRECT)
                    &&  (psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoType != US_E_NFD))
                    || (UssOD_puGetCycSinceLastMeas()[uSnrIdx] > US_D_MAX_NUM_SENSORS))
#endif
            {
                uConfDec = ONE << psSnrPnt->uDecRate;

                if ((psSnrPnt->uDeadPntStat & US_D_DPS_ACCEPTED) != ZERO)
                {
                    if (US_D_OD_KDP_DEAD_PT_CONF < (psSnrPnt->uConf - uConfDec))
                    {
                        psSnrPnt->uConf -= uConfDec;
                        psSnrPnt->uDecRate++;
                    }
                    else
                    {
                        psSnrPnt->uConf = US_D_OD_KDP_DEAD_PT_CONF;
                        psSnrPnt->uDeadPntStat = US_D_DPS_KEPT; /* Mark is as zombie, remove other flags */
                        psSnrPnt->uDecRate = ONE; /* to prevent uConfDec ever overflowing */
                    }
                }
                else
                {
                    if (uConfDec < psSnrPnt->uConf)
                    {
                        psSnrPnt->uConf -= uConfDec;
                        psSnrPnt->uDecRate++;
                    }
                    else
                    {
                        psSnrPnt->uConf = ZERO;
                    }
                }
            }
#endif


#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
            /* New Keep Dead Point (JPP Necro code) */

            /*
             * Only degrade point on it's own direct measurement cycle.
             * GEOMETIC DEGREDATION OF CONFIDENCE ON MEASUREMENT CYCLES  (2^uDecRate)
             * As a safety, if a sensor is not measured in a while, the point
             * will be degraded.
             * Don't degrade on NFD meas cycles, only on direct which should always be active.
             * Note: If you had an NDF only fire seq, points would only degrade every 12 cycles
             *       without adding additional logic.
             */

            if (  (((psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoClass == US_E_DIRECT)
                    &&  (psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoType != US_E_NFD))
                    || (UssOD_puGetCycSinceLastMeas()[uSnrIdx] > US_D_MAX_NUM_SENSORS))
                    && ((psSnrPnt->uDeadPntStat & US_D_DPS_KEPT) == ZERO) ) /* not kept */
            {
                uConfDec = ONE << psSnrPnt->uDecRate;

                if (uConfDec <= psSnrPnt->uConf)
                {
                    psSnrPnt->uConf -= uConfDec;
                    psSnrPnt->uDecRate++;
                }
                else // Confidence at zero. (Point deletion.)
                {
                    /* Keep Dead Points (KDP) acceptance criteria */
                    bIsSideSensor =   (uSnrIdx == US_D_SENSOR_FSR)
                                   || (uSnrIdx == US_D_SENSOR_FSL)
                                   || (uSnrIdx == US_D_SENSOR_RSR)
                                   || (uSnrIdx == US_D_SENSOR_RSL);

                    fY = psSnrPnt->fY;
                    if (fY < 0.0f)
                    {
                        fY = -fY;
                    }
                    if (   (ME_TRUE == bIsSideSensor)
                        && (psSnrPnt->uMaxConf >= US_D_OD_KDP_ACCEPT_CONF)
                        && (fY > 50u)       // Check Y range
                        && (fY < 250u))
                    {
                        UssOD_AddPntToDeadPntBuff(uGrpIdx, psSnrPnt);
                    }

                    /* Delete point from main group list */
                    psSnrPnt->uConf = ZERO;
                }
            }

#else
            /*
             * Point Aging
             * Confidence reduction due to time elapsed between measurements.
             */
            if (   ((psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoClass == US_E_DIRECT)
                    &&  (psSnrCalcs->sSnrCalcRec[uSnrIdx].eEchoType != US_E_NFD))
                    || (UssOD_puGetCycSinceLastMeas()[uSnrIdx] > US_D_MAX_NUM_SENSORS))
            {
                uConfDec = ONE << psSnrPnt->uDecRate;  /* Reduce confidence by 2^decrate */

                if (   (uConfDec <= psSnrPnt->uConf) /* Confidence at or below zero after update */
                    && ((bool_t)ME_FALSE == UssOD_bIsPntInsideVehicle(psSnrPnt->fX, psSnrPnt->fY)))  /* Check if point is in vehicle bounds */
                {
                    psSnrPnt->uConf -= uConfDec;

#if (ME_TRUE == US_D_OD_VERY_SLOW_DEC_FOR_CLOSE_PNTS)
                    if (   (ME_TRUE == bIsNoisy)            /* Noise mode */
                        && (psSnrPnt->uLowDirDist <= 60u))  /* Further closer than 60cm */
                    {
                        psSnrPnt->uDecRate = ONE;  /* Force dec rate to minimum */
                    }
                    else
#endif
                    {
                        psSnrPnt->uDecRate++;  /* Use normal exponential dec rate schedule */
                    }
                }
                else // Confidence at zero. (Point deletion.)
                {
                    psSnrPnt->uConf = ZERO;
                }
            } /* End of point aging condition check */
#endif

#if (US_D_OD_USE_INSTA_KILL_HIGH_CRUISE == ME_TRUE)
            if (psSnrPnt->uCruisingAge >= US_D_OD_INSTA_KILL_CRUISE_VAL)
            {
                psSnrPnt->uConf = ZERO;
            }
#endif
        }  /* End of point non-zero (alive) conf check. */
    } /* Main point list adjustment loop */

    /* Dead point resurrection (Necro - JPP) */
#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
      UssOD_CheckForDeadPntResurrection(uGrpIdx);     // If in FAPA mode, possibly resurrect dead points.
#endif

    /* Clean up - group was cleared by confidence. Now adjust the pnt count*/
    if (eGrpAllowLvl == OD_GRP_CLR_N_SUPPRESS)
    {
        uNumCurPnts = ZERO;
    }

#if (US_D_OD_USE_ODOMETRY_FLAG_FUNC == ME_TRUE) && (US_D_OD_USE_DIRECT_DIST_ODO == ME_TRUE)
    /*
     * Loop through each SENSOR in this group.
     * Presently used for Temporal Cache Odometry
     */
    uNumSnrsInGrp = psPlatPhysInfo->psSnrPhysGrp[uGrpIdx].uNumSnrsInGrp;

    for (uDirSnrGrpIdx = ZERO; uDirSnrGrpIdx < uNumSnrsInGrp; uDirSnrGrpIdx++)
    {
        // Retrieve the index to the sensor, from the physical group data
        uSnrIdx = psPlatPhysInfo->psSnrPhysGrp[uGrpIdx].uSnrDataIdx[uDirSnrGrpIdx];

        /* Pointer to this sensor's cache entry */
        psDirEchoCache =  &sObjDetCalcRec[uSnrIdx].sDirEchoCache;
        uCacheActiveBit = US_D_ECHO_CACHE_L0_ACTIVE;

        /* Loop through cache entries */
        for (uCacheIdx = ZERO; uCacheIdx < US_D_NUM_ECHO_CACHE_ENTRIES; uCacheIdx++)
        {
            /* Adjust cached fixed distance if active */
            if ((psDirEchoCache->uEchoCacheUseStat & uCacheActiveBit) != ZERO)
            {
                psEchoCacheEntry = &psDirEchoCache->sEchoCacheEntry[uCacheIdx];
                psEchoCacheEntry->fLoneSnrAngle -=psOdoAccumOut->fAngleDelta;

                fX = psEchoCacheEntry->fVirtualSnrPosX;
                fY = psEchoCacheEntry->fVirtualSnrPosY;

                psEchoCacheEntry->fVirtualSnrPosX = fX * fCos - fY * fSin;
                psEchoCacheEntry->fVirtualSnrPosY = fX * fSin + fY * fCos;

                // Odo bench unit test/simulation
#if (US_D_TD_ODO_UNIT_TESTING_MODE == 1)
                static float32_t move = 2.0f;
                static float32_t direction = 0.0005f;
                static float32_t limit = 8.0f;

                move += direction;
                if (((direction > 0) && (move > limit)) || ((direction < 0) && (move < -limit)))
                {
                    direction = -direction;
                    move += direction;

                }

                psEchoCacheEntry->fVirtualSnrPosX -= psOdoAccumOut->fXdelta + move;


#else
                psEchoCacheEntry->fVirtualSnrPosX -= psOdoAccumOut->fXdelta;
                psEchoCacheEntry->fVirtualSnrPosY -= psOdoAccumOut->fYdelta;
#endif

            }
            uCacheActiveBit <<= ONE; // Check the next Cache Active bit to the left.
        } /* End of Cache level loop */
    } /* End of group loop */

    /* Perform Odometry on Ego vehicle tracker.
     * Used by flanked ego confidence capping.
     */
    /* Odo for ego flanked confidence capping */
    fX = fMotionTrkX;
    fY = fMotionTrkY;

    fMotionTrkX = fX * fCos - fY * fSin;
    fMotionTrkY = fX * fSin + fY * fCos;

    fMotionTrkX -= psOdoAccumOut->fXdelta;
    fMotionTrkY -= psOdoAccumOut->fYdelta;
#endif

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)
    /* Dead point resurrection debug data */
    _dbg_NecPnt_RTail = uDeadPntRingRightTail[uGrpIdx];
    _dbg_NecPnt_RHead = uDeadPntRingRightHead[uGrpIdx];
    _dbg_NecPnt_LTail = uDeadPntRingLeftTail[uGrpIdx];
    _dbg_NecPnt_LHead = uDeadPntRingLeftHead[uGrpIdx];
#endif

} /* End of AgeAndAdjust function */


#if (ME_TRUE == US_D_OD_DYN_TARG_DEAD_PNT_SWEEPER)
/*===========================================================================
 * @brief Dynamic Target garbage collection. aka Dead Point Sweeper
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks
 */
static void DynTargetDeadPntSweeper(uint8_t uGrpIdx)
{
    uint8_t uNumMeasPnts;
    uint8_t uListIdx;
    uint8_t uPntIdx;
    uint8_t uSnrIdx;
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    uint8_t   uMeasPntListIdx;
    bool_t    bIsSideSensor;
    float32_t fX;
    float32_t fY;
    float32_t fDyingPntX;
    float32_t fDyingPntY;
    float32_t fDx;
    float32_t fDy;
    float32_t fDistSq;
    float32_t fNewPntMinX = 1000.0f;
    US_S_SnrPoint_t * psSnrMeasPnt;
    US_S_SnrPoint_t * psSnrPnt;
    US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();
    uint8_t uMeasPntIdxList[US_D_OD_SWEEPER_NUM_OF_MEAS_PNTS_MAX] = {0};

    uNumMeasPnts = ZERO;  /* Reset count of measured point list */
    /*
     ************* FIRST POINT ANOTATION LOOP ************
     *  Loop through all points in THIS group
     */
    for (uListIdx = 0; uListIdx < uNumCurPnts; uListIdx++)
    {
        uPntIdx = puGrpSnrPntSortIdx[uListIdx]; // Get the sorted point index
        psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];  // Get the record for this sorted point

        /*
         * Dynamic target dead point sweeper, aka dead point garbage collection.
         * Make short list of points measured this cycle
         * Conditions:
         * 1. Confidence >= 30 (US_D_OD_SWEEPER_MIN_CONF_FOR_MEAS_PNT)
         * 2. Cruising age == 0   (Measured this cycle)
         * 3. Not side sensor
         * 4. Point Y value value within vehicle driving tube.
         */

        // Get the sensor who formed this point's index
        uSnrIdx = psSnrPnt->uDirSnrIdx;

        bIsSideSensor =   (bool_t) (   (uSnrIdx == US_D_SENSOR_FSR)
                                    || (uSnrIdx == US_D_SENSOR_FSL)
                                    || (uSnrIdx == US_D_SENSOR_RSR)
                                    || (uSnrIdx == US_D_SENSOR_RSL));

        /* Verify point qualifies for alerts. */
        if (   (ME_FALSE == bIsSideSensor)                                 /* Not a side sensor */
            && ((psSnrPnt->uPntStat & US_D_PNTSTAT_MEASURED) != ZERO)      /* Measured this cycle */
            && (psSnrPnt->uConf >= US_D_OD_SWEEPER_MIN_CONF_FOR_MEAS_PNT)  /* Conf meets threshold */
            && (psSnrPnt->fY >= -psDrivingTubeInfo->fTubeEdgeY)            /* Inside the driving tube */
            && (psSnrPnt->fY <= psDrivingTubeInfo->fTubeEdgeY)
            && (uNumMeasPnts < US_D_OD_SWEEPER_NUM_OF_MEAS_PNTS_MAX))       /* Ensure room exists to store point idx */
        {
           /* Store Measured point index in list, if room */
           uMeasPntIdxList[uNumMeasPnts++] = uPntIdx;

           fX = psSnrPnt->fX;
           if (fX < 0.0f) /* Use only positive numbers */
           {
               fX = -fX;
           }
           if (fNewPntMinX > fX)
           {
               fNewPntMinX = fX;
           }
        }
    }

    /*
     * Perform dead point garbage collection.
     * Sweep up dead points that are adjacent to freshly measured points.
     *
     * Loop through entire list finding dead points that meet conditions:
     * 1. Cruising age >= US_D_OD_SWEEPER_MIN_CRUISE_AGE_FOR_CONF_CAP
     * 2. Conf >= US_D_OD_SWEEPER_CONF_CAP
     * 3. Not side sensor.
     * 4. Meas Point is moving away from bumper and dying pnt in X coord
     *
     * Note: Although this is a nested loop, very worse case is up to 16 remeasured pnts * (num qualifying pnts)
     *       distance checks.  Should be very low number.
     */
    if (uNumMeasPnts != ZERO)  /* Don't bother checking if no new points */
    {
        /* Iterate through existing points */
        for (uListIdx = 0; uListIdx < uNumCurPnts; uListIdx++)
        {
            uPntIdx = puGrpSnrPntSortIdx[uListIdx]; // Get the sorted point index
            psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];  // Get the record for this sorted point

            /* Verify point qualifies for conf cap consideration. */
            if (   (psSnrPnt->uConf >= US_D_OD_SWEEPER_CONF_CAP)
                && (psSnrPnt->uCruisingAge >= US_D_OD_SWEEPER_MIN_CRUISE_AGE_FOR_CONF_CAP))
            {
                /* Perform range and direction test */
                fDyingPntX = psSnrPnt->fX;

                if (fDyingPntX < 0.0f) /* Use only positive numbers */
                {
                    fDyingPntX = -fDyingPntX;
                }

                /* If point is old, dying, has a distance less than the fresh points,
                 * then maybe cap it, if in range. */
                if (fDyingPntX < fNewPntMinX)
                {
                    for (uMeasPntListIdx = ZERO; uMeasPntListIdx < uNumMeasPnts; uMeasPntListIdx++)
                    {
                        psSnrMeasPnt = &sSnrPnt[uGrpIdx][uMeasPntIdxList[uMeasPntListIdx]];  // Get the record for remeasured point
                        fX = psSnrMeasPnt->fX;

                        if (fX < 0.0f) /* Use only positive numbers */
                        {
                            fX = -fX;
                        }

                        /*Calculate X distance between points */
                        fDx = fX - fDyingPntX;

                        /* Only continue if measured point is further away from bumper that dying point */
                        if (   (fDx >= US_D_OD_SWEEPER_X_DIST_MIN)
                            && (fDx <= US_D_OD_SWEEPER_MAX_DIST)) /* For efficiency only. X dist check. */
                        {
                            /* Obtain the Y values for both points */
                            fDyingPntY = psSnrPnt->fY;
                            fY = psSnrMeasPnt->fY;

                            /*Calculate distance between points */
                            fDy = fY - fDyingPntY;
                            fDistSq = fDx * fDx + fDy * fDy;

                            /* Final check to qualify for cap is radial dist check.
                             * Performed using dist squared for efficiency.
                             */
                            if (fDistSq <= US_D_OD_SWEEPER_MAX_DIST_SQ)
                            {
                                /* Found a match.  Cap confidence to non-alert level
                                 * without imposing permanent conf cap
                                 */
                                psSnrPnt->uConf = US_D_OD_SWEEPER_CONF_CAP;
                                break; /* No need to search further for this point */
                            }
                        }
                    }
                }
            }
        } /* End dead point sweeper loop */
    } /* New points exist check */
} /* End dead point sweeper function */
#endif

/*===========================================================================
 * @brief Triangulate each direct echo with each plausible indirect echo,
 * cached direct echo, or create lone-direct estimate.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_PerformPointDetection
 * Remarks:  DD-ID: {B9BDA3DF-5D8A-414a-8CAE-71AD93C4E330}
 * Req.-ID: 17270514
 * Req.-ID: 15001645
 * ===========================================================================*/
/*KPK-QAC Fix : Need to suppress, multiple branch/loop statements doesn't lead to any complexcity */
static void UssOD_PerformPointDetection(uint8_t uGrpIdx)/* PRQA S 2755 */  // Change to UssOD_TriangWIthDirectEchoes  we need to also add direct-direct
{
    const US_S_IndirSnrIdxList_t * psIndirSnrList;
    uint8_t uIndirSnrListIdx;
    uint8_t uDirSnrGrpIdx;
    uint8_t uDirSnrDataIdx;
    uint8_t uIndirSnrDataIdx;
    uint8_t uNumSnrsInGrp;

    bool_t  bIsLeftIndir;
#if 0
    bool_t  bIsDirInnerSensor;
    bool_t  bIsIndirInnerSensor;
#endif
    bool_t  bFirstEchoUsedForTriang;
    bool_t  bIsFakeEcho;

    uint8_t uCurMeasUseStat = US_D_ECHO_USAGE_STAT_UNUSED; // Will be copied into cache later.  Used for echo 0 only.

    const US_S_PlatPhysInfo_t * psPlatPhysInfo = UssCtrl_psGetPlatPhysInfo();
    const US_S_SnrCalcs_t * psSnrCalcs = UssMgr_psGetCurSnrCalcsRec();

    uNumSnrsInGrp = psPlatPhysInfo->psSnrPhysGrp[uGrpIdx].uNumSnrsInGrp;

    /*
     * Perform for each sensor in group, with associated indirect sensors.
     */
    for (uDirSnrGrpIdx = ZERO; uDirSnrGrpIdx < uNumSnrsInGrp; uDirSnrGrpIdx++)
    {
        // Retrieve the index to the sensor, from the physical group data
        uDirSnrDataIdx = psPlatPhysInfo->psSnrPhysGrp[uGrpIdx].uSnrDataIdx[uDirSnrGrpIdx];

        uCurMeasUseStat = US_D_ECHO_USAGE_STAT_UNUSED; // Will be copied into cache later
        bFirstEchoUsedForTriang = ME_FALSE;  // Only applies to echo zero.

        /* Get classification for this direct sensor */
        if (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].eEchoClass == US_E_DIRECT)    // Check if sending sensor (Direct Echo)
        {
            /* Special handling for NFD - Try to create lone direct right away if conditions met. */
//            if ((psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].eEchoType == US_E_NFD) /*|| (psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[ZERO].uDirectDist_cm < 40)*/)
//            {
//                /* Consider if lone direct points should be created for this direct echo */
//
//                UssOD_ConsiderLoneDirects(psSnrCalcs,
//                                          uGrpIdx,
//                                          uDirSnrDataIdx);
//            }
//            else // End: NFD processing

            // Only process Fake NFDs as lones.
            bIsFakeEcho = (bool_t) (!((psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[ZERO].uEchoCalcFlags & (US_D_CALCFLAG_FAKE | US_D_CALCFLAG_HIGH_CONF)) == ZERO));

            {
                /* Obtain list of indirect sensors, for this direct sensor, */
                psIndirSnrList = UssOD_psGetIndirSnrList(uGrpIdx, uDirSnrGrpIdx);

                // Try to triangulate the direct echos with the closest indirect echoes.
                for (uIndirSnrListIdx = ZERO; uIndirSnrListIdx < TWO; uIndirSnrListIdx++)
                {
                    /* Retrieve the index to the indirect echo data */
                    uIndirSnrDataIdx = psIndirSnrList[uIndirSnrListIdx].uIndirSnrDataIdx;

                    /* If no indirect sensor exists, such as at end of group, ignore for triangulation*/
                    bIsLeftIndir =(bool_t)(uIndirSnrListIdx == (uint8_t)US_INDIR_LEFT);

                    // Check if there is no indirect sensor for this direct measurement and it is not a lone fake NFD.
                    // If no indirect skip two sensor triangulation, or if fake nfd, also skip two sensor triangulation
                    if (   (uIndirSnrDataIdx != US_D_OD_NO_INDIR_SNR)
                        && (ME_FALSE == bIsFakeEcho))
                    {

                        /* Attempt to triangulate direct index measurement with indirect index measurement */
#if (US_D_TD_UNIT_TESTING_MODE == 0)
                        bFirstEchoUsedForTriang = UssOD_TriangEchoesDirToIndir(psSnrCalcs,
                                                                               uGrpIdx,
                                                                               uDirSnrDataIdx,
                                                                               uIndirSnrDataIdx,
                                                                               psIndirSnrList[uIndirSnrListIdx].uFovCalcIdx,
                                                                               bIsLeftIndir);

                        if ((bool_t)ME_TRUE == bFirstEchoUsedForTriang)
                        {
                            uCurMeasUseStat |= US_D_ECHO_USAGE_STAT_USED_DI_TRIANG;
                        }
#endif
                        UssOD_UpdateCacheDistanceCalcs(uDirSnrDataIdx);

#if (US_D_TD_UNIT_TESTING_MODE == 0)

                        /* Attempt to triangulate direct index measurement with cached direct index measurement
                         * Will experience temporal delays if not accounted for.  Presently not.
                         * But often saves from creating lone-directs later.
                         */
#if 0
                        bIsDirInnerSensor =   (bool_t)((uDirSnrDataIdx == US_D_SENSOR_FIR)
                                            || (uDirSnrDataIdx == US_D_SENSOR_FIL)
                                            || (uDirSnrDataIdx == US_D_SENSOR_RIR)
                                            || (uDirSnrDataIdx == US_D_SENSOR_RIL));

                        bIsIndirInnerSensor =  (bool_t) ((uIndirSnrDataIdx == US_D_SENSOR_FIR)
                                             || (uIndirSnrDataIdx == US_D_SENSOR_FIL)
                                             || (uIndirSnrDataIdx == US_D_SENSOR_RIR)
                                             || (uIndirSnrDataIdx == US_D_SENSOR_RIL));


                        if (((bool_t)ME_FALSE == bIsDirInnerSensor) || ((bool_t)ME_FALSE == bIsIndirInnerSensor))
#endif
                        {
                            bFirstEchoUsedForTriang = UssOD_TriangEchoesDirToDir(psSnrCalcs,
                                                                                 uGrpIdx,
                                                                                 uDirSnrDataIdx,
                                                                                 uIndirSnrDataIdx, // Is actually the ID of the cached direct index.
                                                                                 psIndirSnrList[uIndirSnrListIdx].uFovCalcIdx,
                                                                                 bIsLeftIndir);
                            if (ME_TRUE == bFirstEchoUsedForTriang)
                            {
                                uCurMeasUseStat |= US_D_ECHO_USAGE_STAT_USED_DD_TRIANG;
                            }
                        }
#endif
                    } // End: there is a an indirect sensor for this direct
                } // End loop indirect list for given direct sensor measurement

                /* If echo is thus far unused and is not a fake NFD then try temporal triang */
                if  (   (uCurMeasUseStat == US_D_ECHO_USAGE_STAT_UNUSED)
                     && (ME_FALSE == bIsFakeEcho))
                {
                    bFirstEchoUsedForTriang = UssOD_TriangEchoesTemporalDir(psSnrCalcs,
                                                                            uGrpIdx,
                                                                            uDirSnrDataIdx);
                    if ((bool_t)ME_TRUE == bFirstEchoUsedForTriang)
                    {
                        uCurMeasUseStat |= US_D_ECHO_USAGE_STAT_USED_TD_TRIANG;
                    }
                }

                /* If this echo hasn't been used, we are out of options for this direct, for now.
                 * Create weak lone direct anyway, or match, or ignore.
                 */
                if (uCurMeasUseStat == US_D_ECHO_USAGE_STAT_UNUSED)
                {
                    UssOD_ConsiderLoneDirects(psSnrCalcs,
                                              uGrpIdx,
                                              uDirSnrDataIdx);
                }

                UssOD_UpdateCacheEntries(psSnrCalcs, uDirSnrDataIdx, uCurMeasUseStat);
            } // End: Direct measurement (non-NFD)  processing
        }  // End: There IS a new measurement for this direct sensor
    } // End: Loop through direct sensors in this group
}

static void UssOD_UpdateCacheEntries(const US_S_SnrCalcs_t *psSnrCalcs,
                                     uint8_t uDirSnrDataIdx,
                                     uint8_t uCurMeasUsageStat)
{
    const US_S_SnrAngleCalcs_t * psSnrAngleCalcs  = UssOD_psGetSnrAngleCalcs();   /* Get angle calculations */
    const US_S_EchoCalc_t *sEchoCalc = &psSnrCalcs->sSnrCalcRec[uDirSnrDataIdx].sEchoCalc[ZERO];
    US_S_DirEchoCache_t *psDirEchoCache = &sObjDetCalcRec[uDirSnrDataIdx].sDirEchoCache;
    US_S_EchoCacheEntry_t *psEchoCacheEntry;
    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;
    const uint32_t uCurTimestamp = US_GetTimeStampTimeMs();

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }

    if(pSnrCoord == NULL)
    {
        return;   // Check if data exists.  Early return for functional safety.
    }

    uint8_t uCacheStat = psDirEchoCache->uEchoCacheUseStat;

    /* Just for safety */
    if ((uCacheStat & US_D_ECHO_CACHE_L1_ACTIVE) == ZERO) // L0 Not Active
    {
        uCacheStat &= (uint8_t)~US_D_ECHO_CACHE_L1_LATCHED;
    }

    /* If L2 to far away, invalidate the cache entry. */
    if ((uCacheStat & US_D_ECHO_CACHE_L2_ACTIVE) != ZERO) // L2 Active
    {
        if (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L2_FAR].fPairDist  >= (float32_t) US_D_CACHE_MID_SNRDIST_MAX)
        {
            if (   ((uCacheStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)    // L0 Active
                || ((uCacheStat & US_D_ECHO_CACHE_L1_ACTIVE) != ZERO))   // L1 Active
            {
                uCacheStat &= (uint8_t)~US_D_ECHO_CACHE_L2_ACTIVE;
            }
        }
    }

    /* If L2 not valid but L1 is, then copy L1 to L2, then un-latch L1 so it drops a fresh breadcrumb */
    if (  ((uCacheStat & US_D_ECHO_CACHE_L1_ACTIVE) != ZERO)    // L1 Active
        && ((uCacheStat & US_D_ECHO_CACHE_L2_ACTIVE) == ZERO))  // L2 Not Active
    {
        // l2 = l1
        (void)memcpy(&psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L2_FAR],
               &psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L1_MID],
               sizeof(US_S_EchoCacheEntry_t));

        uCacheStat |= US_D_ECHO_CACHE_L2_ACTIVE;
        uCacheStat &= (uint8_t)~US_D_ECHO_CACHE_L1_LATCHED;
    }

    /* If L2 is still not valid, try to copy L0 */
    if (   ((uCacheStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)  // L0 Active
        && ((uCacheStat & US_D_ECHO_CACHE_L2_ACTIVE) == ZERO)) // L2 Not Active
    {
        // l2 = l0
        (void)memcpy(&psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L2_FAR],
               &psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV],
               sizeof(US_S_EchoCacheEntry_t));

        uCacheStat |= US_D_ECHO_CACHE_L2_ACTIVE;
    }

    /* If L1 is un-latched, update it. */
    if ((uCacheStat & US_D_ECHO_CACHE_L1_LATCHED) == ZERO) // L1 Limit not yet reached.
    {
        /* Update from L0 if active */
        if ((uCacheStat & US_D_ECHO_CACHE_L0_ACTIVE) != ZERO)    // L0 Active
        {
            /* Only improve on L1, if Active, by a greater distance L0 */
            if (   (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].fPairDist  >=
                         psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L1_MID].fPairDist)
                || ((uCacheStat & US_D_ECHO_CACHE_L1_ACTIVE) == ZERO))   // L1 Not active
            {
                (void)memcpy(&psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L1_MID],
                       &psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV],
                       sizeof(US_S_EchoCacheEntry_t));

                uCacheStat |= US_D_ECHO_CACHE_L1_ACTIVE;
            }

        }
#if 0
        else /* If L0 is not active than invalidate L1 */
       {
              uCacheStat &= ~US_D_ECHO_CACHE_L1_ACTIVE; 
        } 
#endif
        /* Check if L1 needs to be latched ie. drop a breadcrumb */
        if (   ((uCacheStat & US_D_ECHO_CACHE_L2_ACTIVE) != ZERO)           // L2 Active
            && (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L2_FAR].fPairDist >= (float32_t)US_D_CACHE_MID_SNRDIST_LIM))  // Min Dist reached.
        {
            uCacheStat |= US_D_ECHO_CACHE_L1_LATCHED;
        }
    }

    /*
     * Update L0 with current measurement manually.
     */
    psEchoCacheEntry = &psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV];

    /* Only update for non-zero measurements, or measurements that show some movement */
    if (   (sEchoCalc->uDirectDist_cm > ZERO)
        && ((sEchoCalc->uEchoCalcFlags & US_D_CALCFLAG_FAKE) == ZERO)) // Check if fake NFD.  If so, don't cache.
    {

        /* A measurement exist and thus is stored in the L0 cache */
#if (US_D_TD_CACHE_UNIT_TESTING_MODE == ME_TRUE)
        psEchoCacheEntry->uDirDist        = ONE;
#else
        psEchoCacheEntry->uDirDist        = sEchoCalc->uDirectDist_cm;
#endif
        psEchoCacheEntry->uSignalStrength = sEchoCalc->uSignalStrength;
        psEchoCacheEntry->bLargeObj       = sEchoCalc->bLargeObject;

        psEchoCacheEntry->fVirtualSnrPosX = pSnrCoord[uDirSnrDataIdx].fX;
        psEchoCacheEntry->fVirtualSnrPosY = pSnrCoord[uDirSnrDataIdx].fY;

        psEchoCacheEntry->fLoneSnrAngle   = psSnrAngleCalcs->fSnrAngInVehCoords_rad[uDirSnrDataIdx]; // Angle of sensor in vehicle coord system.
        psEchoCacheEntry->uMeasUseStat    = uCurMeasUsageStat;

        /*  Check for Ego movement for standstill timeout */
        if (psDirEchoCache->sEchoCacheEntry[US_D_ECHO_CACHE_L0_PREV].fPairDist >= 0.0001f) /* ego shows movement */
        {
            if (ZERO != psDirEchoCache->uTimeoutOfEchoCache) /* No timeout presently set */
            {
                /* Timeout of L2/L3 cache due to extended standstill */
                if (uCurTimestamp > psDirEchoCache->uTimeoutOfEchoCache)
                {
                    psDirEchoCache->uEchoCacheUseStat = US_D_ECHO_CACHE_STAT_IDLE; /* Clear cache */
                    psDirEchoCache->uTimeoutOfEchoCache = ZERO;  /* Shut off standstill timer */
                }
            }
        }
        else /* Ego is standing still */
        {
            if (ZERO == psDirEchoCache->uTimeoutOfEchoCache) /* No timeout presently set */
            {
                /* Set the timeout expiration timestamp */
                if (uCurTimestamp > (UINT_MAX - US_D_CACHE_STANDSTILL_TIMEOUT))
                {
                  /* Handle potential wrap-around, which will only happen if driving for hours */
                     psDirEchoCache->uTimeoutOfEchoCache = (uCurTimestamp - (UINT_MAX - US_D_CACHE_STANDSTILL_TIMEOUT)) - 1u;
                }
                else
                {
                    psDirEchoCache->uTimeoutOfEchoCache = uCurTimestamp + US_D_CACHE_STANDSTILL_TIMEOUT;
                }
            }
        }

        /* Update cache status */
        psDirEchoCache->uEchoCacheUseStat = uCacheStat | (uint8_t) US_D_ECHO_CACHE_L0_ACTIVE;
    }
}

static void UssOD_UpdateCacheDistanceCalcs(uint8_t uDirSnrDataIdx)
{
    US_S_DirEchoCache_t *psDirEchoCache = &sObjDetCalcRec[uDirSnrDataIdx].sDirEchoCache;
    US_S_EchoCacheEntry_t *psEchoCacheEntry;
    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;
    float32_t fDeltaX;
    float32_t fDeltaY;
    float32_t fVirtSnrDist;
    uint8_t uCacheActiveBit;
    uint8_t uCacheIdx;

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }

    if(pSnrCoord == NULL)
    {
        return;   // Check if data exists.  Early return for functional safety.
    }

    psDirEchoCache = &sObjDetCalcRec[uDirSnrDataIdx].sDirEchoCache;

    uCacheActiveBit = US_D_ECHO_CACHE_L0_ACTIVE;

    // Loop through caches in L3->L1 order
    for (uCacheIdx = ZERO; uCacheIdx < US_D_NUM_ECHO_CACHE_ENTRIES; uCacheIdx++)
    {
        // Retrieve the echo cache entry for this sensor and cache depth.
        psEchoCacheEntry = &psDirEchoCache->sEchoCacheEntry[uCacheIdx];

        /* Adjust cached fixed distance if active */
        if ((psDirEchoCache->uEchoCacheUseStat & uCacheActiveBit) != ZERO)
        {
            // Retrieve the echo cache entry for this sensor and cache depth.
            psEchoCacheEntry = &psDirEchoCache->sEchoCacheEntry[uCacheIdx];

            /* Check length of sensor pair */
            fDeltaX = psEchoCacheEntry->fVirtualSnrPosX - pSnrCoord[uDirSnrDataIdx].fX;
            fDeltaY = psEchoCacheEntry->fVirtualSnrPosY - pSnrCoord[uDirSnrDataIdx].fY;

            /* Calculate distance between sensors */
            fVirtSnrDist = (float32_t)sqrtf(fDeltaX * fDeltaX + fDeltaY * fDeltaY); // Calculate distance between sensors.

            if (uCacheIdx != US_D_ECHO_CACHE_L0_PREV)
            {
                if (fVirtSnrDist > (float32_t)US_D_CACHE_SNRDIST_L2_L3_MAXLIM)
                {
                    psDirEchoCache->uEchoCacheUseStat &= (uint8_t)~uCacheActiveBit;
                }
            }
            else
            {
                if (fVirtSnrDist >(float32_t) US_D_CACHE_SNRDIST_L0_MAXLIM)
                {
                    psDirEchoCache->uEchoCacheUseStat &= (uint8_t)~uCacheActiveBit;
                }
            }

            /* Store these for later use */
            psEchoCacheEntry->fPairDist = fVirtSnrDist;
            psEchoCacheEntry->fDeltaX   = fDeltaX;
            psEchoCacheEntry->fDeltaY   = fDeltaY;
        }
        uCacheActiveBit <<= ONE;
    }

}

#define US_D_OD_PNT_BUFFER_UNDERFLOW  (0xFFu)

/*===========================================================================
 * @brief Sort and merge old and new points into final point list for group.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks Expectation is the the point map buffer for this group has been
 *          sorted by confidence level, high to low and new points have been
 *          added from the bottom of the list upward, overwriting existing
 *          points if necessary.
 *Req.-ID: 13517239
 *Req.-ID: 13420920
 */
/* ===========================================================================
 * Name: UssOD_SortAndMergeFinalList	
 * Remarks:  DD-ID: {3F7CCD58-3E13-422b-B774-D6C61D71A360}
 * ===========================================================================*/
static void UssOD_SortAndMergeFinalList(uint8_t uGrpIdx)
{

#if (US_D_OD_USE_FAST_SORT == ME_TRUE)
    uint8_t uPntCnt;
    uint8_t uNewPntIdx;
    uint8_t uExistingPntIdx;
    uint8_t uTotNumPnts = uNumNewPnts + uNumCurPnts;
    uint8_t uDestPntIdx;
    US_S_SnrPoint_t * psDestSnrPnt;
    US_S_SnrPoint_t * psSnrPnt;
    US_S_SnrPoint_t * psNewSnrPnt;
    US_S_SnrPoint_t * psSnrPntGrp = sSnrPnt[uGrpIdx];
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
#endif
    UssOD_SortPointsByConfidence(sSnrPnt[uGrpIdx],
                                 uSnrPntSortIdx[uGrpIdx],
                                 ZERO,
                                 uNumCurPnts);              // Sort old points with adjusted confidences.

#if (US_D_OD_USE_FAST_SORT == ME_TRUE)
    if (uTotNumPnts < US_D_OD_MAX_PNTS_FOR_SEP_SORT)
    {
        /*
         * Sort and merge new and existing point lists.
         */
        UssOD_SortPointsByConfidence(sSnrNewPntBuff,
                                     uSnrNewPntSortIdx,
                                     ZERO,
                                     uNumNewPnts);

        /* Set initial indices and pointers for merge of lists */
        uNewPntIdx = uNumNewPnts - ONE;
        uExistingPntIdx = uNumCurPnts - ONE;
        uDestPntIdx = uTotNumPnts - ONE;

        /* Get pointer to the first location to store merge point list,
         * started at the end of old and new points total,
         * of the sorted existing point list.
         */
        psDestSnrPnt =  &psSnrPntGrp[puGrpSnrPntSortIdx[uDestPntIdx]];

        /* Starting with the last existing point, we will be stepping backward through the group's points.
         * Retrieve pointer to the last existing point
         */
        psSnrPnt = &psSnrPntGrp[puGrpSnrPntSortIdx[uExistingPntIdx]];

        /* Loop through all new points in new point list,
         * if none then the existing point list is complete by default.
         */
        for (uPntCnt = ZERO; uPntCnt < uNumNewPnts; uPntCnt++)
        {
            /* Get pointer to last new point, again we will progress backwards through them,
             * for as many new points as they are.
             */
            psNewSnrPnt = &sSnrNewPntBuff[uSnrNewPntSortIdx[uNewPntIdx]];

            /*
             * If existing point has a lower confidence then new point then store in next
             * location for point list.
             */
            while (   (uExistingPntIdx != US_D_OD_PNT_BUFFER_UNDERFLOW)
                   && (uDestPntIdx != US_D_OD_PNT_BUFFER_UNDERFLOW)
                   && (psSnrPnt->uConf < psNewSnrPnt->uConf))
            {
                // If old index does not match dest idx then place old point in new location
                if (uDestPntIdx != uExistingPntIdx)
                {
                    memcpy(psDestSnrPnt, psSnrPnt, sizeof(US_S_SnrPoint_t));
                }

                uDestPntIdx--;
                if (uDestPntIdx != US_D_OD_PNT_BUFFER_UNDERFLOW) // Should never occur if algorithm does not have bug.
                {
                    psDestSnrPnt =  &psSnrPntGrp[puGrpSnrPntSortIdx[uDestPntIdx]];
                }

                uExistingPntIdx--;
                if (uExistingPntIdx != US_D_OD_PNT_BUFFER_UNDERFLOW)
                {
                    psSnrPnt = &psSnrPntGrp[puGrpSnrPntSortIdx[uExistingPntIdx]];
                }
            }

            /* Merge new point into existing point list */
            memcpy(psDestSnrPnt, psNewSnrPnt,sizeof(US_S_SnrPoint_t));

            /* Adjust next location to store points */
            uDestPntIdx--;
            if (uDestPntIdx != US_D_OD_PNT_BUFFER_UNDERFLOW) // Should never occur if algorithm does not have bug.
            {
                psDestSnrPnt =  &psSnrPntGrp[puGrpSnrPntSortIdx[uDestPntIdx]];
            }

            uNewPntIdx--;
        }

        uNumCurPnts = uTotNumPnts;

        /* By default, remainder of existing point list will already be stored in the correct
         * locations in group point list.
         */
    }
    else
#endif /* if US_D_OD_USE_FAST_SORT == ME_TRUE */
    {

        /* Proceed by added new points to existing point list
         * from the bottom up and sort full merged list.
         * (This is the original algo before above optional efficiency check,
         *  and alternate merge method added.)
         */

        UssOD_AddNewPoints(uGrpIdx);  // Add new points at the bottom of old points list.

        UssOD_SortPointsByConfidence(sSnrPnt[uGrpIdx],
                                     uSnrPntSortIdx[uGrpIdx],
                                     ZERO,
                                     US_D_POINT_BUFFER_SIZE);  // Sort performed to determine best points for publishing and next OD iteration.

    }


    uNumNewPnts = ZERO; // Clear new point list in preparation for next cycle.
}

/*===========================================================================
 * @brief Add newly triangulated points to phys group point map.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks Expectation is the the point map buffer for this group has been
 *          sorted by confidence level, high to low.
 */
/* ===========================================================================
 * Name:	 UssOD_AddNewPoints
 * Remarks:  DD-ID: {59A2A5E6-298A-4761-A620-27175BD41DF3}
 * Traceability to source Code: Req.-ID: 15002839, 17270853
 *  Req.-ID:  17270853
 * ===========================================================================*/
static void UssOD_AddNewPoints(uint8_t uGrpIdx)
{
    uint8_t uListIdx = US_D_POINT_BUFFER_SIZE - ONE; // Start at end of point list
    uint8_t uPntIdx;
    uint8_t uNewPntIdx;
    US_S_SnrPoint_t * psSnrPnt;
    US_S_SnrPoint_t * psNewSnrPnt;

    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    US_S_SnrPoint_t * psSnrPntGrp = sSnrPnt[uGrpIdx];

    for (uNewPntIdx = ZERO; uNewPntIdx < uNumNewPnts; uNewPntIdx++)
    {
        /* Start storing at bottom of list, which will decrement */
        uPntIdx = puGrpSnrPntSortIdx[uListIdx];
        psSnrPnt = &psSnrPntGrp[uPntIdx];
        psNewSnrPnt = &sSnrNewPntBuff[uNewPntIdx];

        (void)memcpy(psSnrPnt, psNewSnrPnt, sizeof(US_S_SnrPoint_t));

        /*
         * Continue storing backward through sorted point list
         * Note:  Points of lower priority could potentially overwrite higher priority
         *        points at the very bottom of the list.  But all new points are given a "chance",
         *        and if the list is deep enough, this should never be an issue.
         */
        uListIdx--;

        /* Should never happen.  For buffer underflow safety */
        if (uListIdx == US_D_OD_PNT_BUFFER_UNDERFLOW)
        {
            break;  // In case of this, just don't add any more points.
        }
    }

    /* Update number of points in list (not yet sorted) */
    uNumCurPnts += uNumNewPnts;
    if (uNumCurPnts > US_D_POINT_BUFFER_SIZE)
    {
        uNumCurPnts = US_D_POINT_BUFFER_SIZE;
    }

}

/*===========================================================================
 * @brief Modify points based on their state AFTER new points are added
 *
 * @param uint8_t uGrpIdx
 * @param
 * @return
 * @remarks Mainly designed to handle points that have moved too far from
 *          their origin
 */
/* ===========================================================================
 * Name:	 UssOD_AdjustTrackedPoints
 * Remarks:  DD-ID: 
 * Traceability to source Code: 
 * ===========================================================================*/
#if (US_D_OD_USE_CR_DIST_KILL == ME_TRUE) /* function might be used for other items */
static void UssOD_AdjustTrackedPoints(uint8_t uGrpIdx)
{
    uint8_t uListIdx;
    uint8_t uPntIdx;
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    US_S_SnrPoint_t * psSnrPnt;
    float32_t fDistDiff;
    float32_t fDistLimit;
    float32_t fDistLimitNeg; /* using RAM to save CPU */
    bool_t bDeleteThisPoint;
#if (US_D_OD_USE_CRDK_IGNORE_CLOSE == ME_TRUE)
    const US_S_SensorsCfg_t * pSnrCfg = US_SnrCfg_F_Get_SnrCfg();   // Get calibration record.
    const US_S_Sensor_coordinate_t * pSnrCoord = NULL;

    /* Get the address of Sensor Coordinate from Calibration data */
    if (pSnrCfg != NULL)
    {
        pSnrCoord = pSnrCfg->pSnrCoordinate;
    }
#endif
    /* Get the settings for this group */
    switch(uGrpIdx)
    {
        case US_D_PHYS_GRP_REAR:
        {
            fDistLimit = US_D_OD_USE_CRDK_REAR_DIST_CM;
            fDistLimitNeg = -US_D_OD_USE_CRDK_REAR_DIST_CM;
            break;
        }
        case US_D_PHYS_GRP_FRNT:
        default: /* Covers side groups if those happen */
        {
            fDistLimit = US_D_OD_USE_CRDK_FRNT_DIST_CM;
            fDistLimitNeg = -US_D_OD_USE_CRDK_FRNT_DIST_CM;
            break;
        }
    }

    /* Loop through all the points... again */
    for (uListIdx = 0; uListIdx < uNumCurPnts; uListIdx++)
    {
        uPntIdx = puGrpSnrPntSortIdx[uListIdx]; // Get the sorted point index
        psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];  // Get the record for this sorted point
        bDeleteThisPoint = ME_FALSE;

        if (psSnrPnt->uConf != ZERO)  // Check if valid point
        {
            /* NOTE: Only doing X direction, should be a additive of X & Y later */
            if ( (psSnrPnt->fAccuX > fDistLimit) || (psSnrPnt->fAccuX < fDistLimitNeg) )
            {
#if (US_D_OD_USE_CRDK_IGNORE_CLOSE == ME_TRUE)
                /* Point has gone too far without a remeasurement - see if it's too close to the car to care */
                /* Can't used psSnrPnt->uLowDirDist, this is dist when MEASURED */
                if ((pSnrCoord != NULL) && (US_D_OD_USE_CRDK_CLOSE_CM > 0.0f))
                {
                    /* use snr idx for direct echo for the point as ref*/
                    fDistDiff = ((psSnrPnt->fX) - (pSnrCoord[psSnrPnt->uDirSnrIdx].fX));
                    
                    if ( (fDistDiff > US_D_OD_USE_CRDK_CLOSE_CM) || (fDistDiff < -US_D_OD_USE_CRDK_CLOSE_CM) )
                    {
                        bDeleteThisPoint = ME_TRUE;
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
                        uDBG_DeleteReason = 0x11;
#endif
                    }
                    else
                    {
                        /* basically related deletion due to being too close*/
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
                        uDBG_DeleteReason = 0x33;
#endif
                    }
                }
                else
#endif
                {
                    /* cal data bad or we don't care.. skip the distance check*/
                    bDeleteThisPoint = ME_TRUE;
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
                    uDBG_DeleteReason = 0x22;
#endif
                }

                /* do we care about cruising age or other factors? */

                /* delete the point */
                if ((bool_t)ME_TRUE == bDeleteThisPoint)
                {
                    psSnrPnt->uConf = ZERO; //is this the right way? might need more clean up

#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
                    /* Debug */
                    uTravelKillCounter++; //debug counter
                    siLastKill_X = (sint16_t)psSnrPnt->fX;
                    siAccuKill_X = (sint16_t)psSnrPnt->fAccuX;
                    uDBG_NumGrpPoints = uNumCurPnts;
                    uDBG_InPtIdx = uPntIdx;
                    uDBG_ListPtIdx = uListIdx;
#endif
                }
            }
        }
    }
}
#endif /* #if (US_D_OD_USE_CR_DIST_KILL == ME_TRUE) */
/*===========================================================================
 * @brief Check if the group needs to be cleared and ignored
 *
 * @param uint8_t uGrpIdx
 * @param bool_t bUpdate: should the group state/counts be updated
 * @return
 * @remarks Originaly created for handling noisy mode
 */
/* ===========================================================================
 * Name:	 UssOD_GetAndUpdateGrpAllowLevel
 * Remarks:  DD-ID: {A7BFD905-FBE8-4ac6-A964-2EE5560C9559}
 * Traceability to source Code: 
 * ===========================================================================*/
static US_E_Grp_Allow_Level_t UssOD_GetAndUpdateGrpAllowLevel(uint8_t uGrpIdx, US_E_Dir_Ego_Motion_t eCurDirOfMotion)
{
    UssMgr_E_GroupState_t eCurGrpState;
    US_E_Grp_Allow_Level_t eRetGrpAllow = OD_GRP_ALL_ALLOW;

    /* Update state and counters */

#if (US_D_OD_EGO_DIR_CHANGE_CONF_REDUCE == ME_TRUE)

    /* If previous motion and current motion are verified */
    if (   (eCurDirOfMotion != OD_DIR_EGO_MOT_UNKNOWN)
        && (ePrevDirOfActiveMotion != OD_DIR_EGO_MOT_UNKNOWN))
    {
        /* On direction change */
        if (eCurDirOfMotion != ePrevDirOfActiveMotion)
        {
            if ((eCurDirOfMotion == OD_DIR_EGO_MOT_D) && (uGrpIdx == US_D_PHYS_GRP_REAR))
            {
                uSuppressCounter[uGrpIdx] = US_D_OD_USE_GSK_CYCLES;
                eGrpAllowLevel[uGrpIdx] = OD_GRP_REDUCE_CONF;
            }
            else if ((eCurDirOfMotion == OD_DIR_EGO_MOT_R) && (uGrpIdx == US_D_PHYS_GRP_FRNT))
            {
                uSuppressCounter[uGrpIdx] = US_D_OD_USE_GSK_CYCLES;
                eGrpAllowLevel[uGrpIdx] = OD_GRP_REDUCE_CONF;
            }
            else /* Prev or current direction of motion unconfirmed */
            {
                /* Ignore, don't conf reduce anything */
            }
        }
    }
#endif /* #if (US_D_OD_EGO_DIR_CHANGE_CONF_REDUCE == ME_TRUE) */

    eCurGrpState = UssMgr_eGetGrpState(uGrpIdx);

#if (US_D_OD_USE_NOISY_SWITCH_KILL == ME_TRUE)
#if (US_D_OD_USE_NSK_ONLY_FRONT == ME_TRUE)
    if ( (uGrpIdx == US_D_PHYS_GRP_FRNT) && (eCurGrpState != g_prevGrpState[uGrpIdx]) && (eCurGrpState == SYSMGR_GRPSTATE_NOISY) )
#else
    if ( (eCurGrpState != g_prevGrpState[uGrpIdx]) && (eCurGrpState == SYSMGR_GRPSTATE_NOISY) )
#endif
    {
        //bSuppressGrp[uGrpIdx] = ME_TRUE;
        uSuppressCounter[uGrpIdx] = g_uSuppressNoisyThresh; //US_D_OD_USE_NSK_CYCLES
        eGrpAllowLevel[uGrpIdx] = OD_GRP_CLR_N_SUPPRESS;
    }
#endif


    /* Making this independent prevents a suppress counter of zero going forever */
    if(uSuppressCounter[uGrpIdx] == ZERO)
    {
        eGrpAllowLevel[uGrpIdx] = OD_GRP_ALL_ALLOW;
    }
    /* Count down AFTER determining all allow conditions, guarantees set level persists 1 cycle minimum */
    if(uSuppressCounter[uGrpIdx] > ZERO)
    {
        uSuppressCounter[uGrpIdx]--;
    }

    /* End of updating, clean up */
    /* Save previous state */
    g_prevGrpState[uGrpIdx] = eCurGrpState;

    /* Send saved allow level */
    eRetGrpAllow = eGrpAllowLevel[uGrpIdx];
    return eRetGrpAllow;
}

static US_E_Dir_Ego_Motion_t eGetCurVehDirOfMotion(void)
{
    US_E_Dir_Ego_Motion_t eCurDirMotion = OD_DIR_EGO_MOT_UNKNOWN;
    UssOdo_OdoAccumOutput_t * psOdoAccumOut;
    US_E_App_Prndl_Status_e ePrndl;
    float32_t fVehSpeed_kph = UssCom_F_GetVehSpeed();

    /* Confirm vehicle motion */
    if (fVehSpeed_kph > 0.0f)
    {
        ePrndl = UssCom_F_GetVehPrndl();
        psOdoAccumOut = UssOdo_psGetOdoAccumOut();

        /* Confirm motion in the direction of travel */
        if ((ePrndl == US_PRNDL_DRIVE) && (psOdoAccumOut->fXdelta > 0.0f))  /* 0.01 cm in 40ms is 0.009 km or 0.0056 mph */
        {
            eCurDirMotion = OD_DIR_EGO_MOT_D;
        }
        else if ((ePrndl == US_PRNDL_REVERSE) && (psOdoAccumOut->fXdelta < 0.0f))
        {
            eCurDirMotion = OD_DIR_EGO_MOT_R;
        }
        else
       {
	/* Nothing to do here, by design. */
       }
    }

    return eCurDirMotion;
}

#ifdef US_D_OD_HEIGHT_GEOM_BG_NOISE_TRACK
float32_t g_fBackgroundNoiseLevel = 0.0f; 
void UssOD_ComputeBackgroundNoiseLevel(uint8_t uGrpIdx)
{
    uint16_t uCount = 0; 
    uint8_t uListIdx;
    uint8_t uPntIdx;
    US_S_SnrPoint_t * psSnrPnt;
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];

    /* TBD, fix. for now, this function will only run for rear group */
    if (uGrpIdx != US_D_PHYS_GRP_REAR) return;

    for (uListIdx = 0; uListIdx < uNumCurPnts; uListIdx++)
    {
        uPntIdx = puGrpSnrPntSortIdx[uListIdx]; // Get the sorted point index
        psSnrPnt = &sSnrPnt[uGrpIdx][uPntIdx];  // Get the record for this sorted point

        // If confidence of point is above threshold, proceed
        if (psSnrPnt->uConf > US_D_OD_HEIGHT_GEOM_BG_NOISE_THRESH) 
        {
            continue; 
        }
        else if (psSnrPnt->uConf <=  ZERO) /* exit sorted list when no more points exist*/
        {
            break;
        }
        else if (psSnrPnt->uCruisingAge == ZERO) /* if it is a new point where 0 < uConf <= US_D_OD_HEIGHT_GEOM_BG_NOISE_THRESH then */
        {
            /* increment counter */
            uCount++; 
        }
        else 
		{ 
		            //Do Nothing
		}
    }

    /* update moving average */
    g_fBackgroundNoiseLevel = (g_fBackgroundNoiseLevel*US_D_OD_HEIGHT_GEOM_BG_NOISE_ALPHA) + (float32_t)(uCount); 

}
#endif //US_D_OD_HEIGHT_GEOM_STATS

/*===========================================================================
 * @brief Reset the measurement status for all points in this group.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks Expectation is the the point map buffer for this group has been
 *          sorted by confidence level, high to low.
 */
/* ===========================================================================
 * Name:	 UssOD_ResetPntMeasStat
 * Remarks:  DD-ID: {15E1E37E-A4D6-432d-A298-95F7F302002E}
 * Req.-ID: 13519880
 * ===========================================================================*/
static void UssOD_ResetPntMeasStat(uint8_t uGrpIdx)
{
    uint8_t uListIdx;

    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    US_S_SnrPoint_t * psSnrPntGrp = sSnrPnt[uGrpIdx];

    uNumCurPnts = ZERO;
    for (uListIdx = ZERO; uListIdx < US_D_POINT_BUFFER_SIZE; uListIdx++)
    {
        if (psSnrPntGrp[puGrpSnrPntSortIdx[uListIdx]].uConf > ZERO)
        {
            psSnrPntGrp[puGrpSnrPntSortIdx[uListIdx]].uPntStat &= US_D_PNTSTAT_PERSIST_MASK;   // Make us not measured this cycle, so far.
            uNumCurPnts++;
        }
        else
        {
            psSnrPntGrp[puGrpSnrPntSortIdx[uListIdx]].uPntStat = US_D_PNTSTAT_NULL; // We are a dead point.
        }
    }
}

/*===========================================================================
 * @brief Reset the status of usage of measurements by object detection.
 *
 * @param bool_t bInitTriangDirDirStat - if true to full initialization reset.
 * @param
 * @return
 * @remarks bool_t bInitTriangDirDirStat - if false just reset triangulation status.
 */
/* ===========================================================================
 * Name:	 UssOD_ResetOdCalcs
 * Remarks:  DD-ID: {CFC90F57-01CF-46cb-BA66-8A73E8511C8B}
 * ===========================================================================*/
void UssOD_ResetOdCalcs(bool_t bInitTriangDirDirStat)
{
    uint8_t uSnrIdx;
    US_S_DirEchoCache_t *psDirEchoCache;

    for (uSnrIdx = ZERO; uSnrIdx < US_D_MAX_NUM_SENSORS; uSnrIdx++)     // Scan all sensor positions
    {
        psDirEchoCache =  &sObjDetCalcRec[uSnrIdx].sDirEchoCache;

         /* If true, do an unconditional reset */
        if (bInitTriangDirDirStat == (bool_t)ME_TRUE)
        {
            psDirEchoCache->uTimeoutOfEchoCache = ZERO;
            psDirEchoCache->uEchoCacheUseStat = US_D_ECHO_CACHE_STAT_IDLE;

        }

        sObjDetCalcRec[uSnrIdx].bNearFieldSeen = ME_FALSE;
    }
}

static bool_t UssOD_bIsPntInsideVehicle(float32_t fX, float32_t fY)
{
    US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();
    bool_t bResult = ME_FALSE;

    if ((fX <= psDrivingTubeInfo->fFrontBumperX) && (fX >= psDrivingTubeInfo->fRearBumperX))
    {
        if ((fY <= psDrivingTubeInfo->fTubeEdgeY) && (fY >= -psDrivingTubeInfo->fTubeEdgeY))
        {
            bResult = ME_TRUE;
        }
    }

    return (bResult);
}

#if (US_D_OD_CAP_PNT_CONF_WHEN_FLANKED == ME_TRUE)
/* Cap the confidence for points in the driving tube when
 * flanked by garage doors or similar.
 */
/* ===========================================================================
 * Name:UssOD_CapConfForPntsWhenFlanked	
 * Remarks:  DD-ID: {ED803D38-ABD3-43f8-8C7C-9AF75CCDF790}
 * ===========================================================================*/

void UssOD_CapConfForPntsWhenFlanked(uint8_t uGrpIdx)
{
    static bool_t bFirstActiveCycle = ME_TRUE;
    uint8_t uPntIdx;
    uint8_t uListIdx;
    uint8_t uDirSnrDataIdx;
    uint8_t uMinConfToQualify;
    uint8_t uTubePntCnt = ZERO;
    bool_t bIsSideSensor;
    bool_t bIsFlankedRight = ME_FALSE;
    bool_t bIsFlankedLeft  = ME_FALSE;
    bool_t bUseThisBus = ME_FALSE;
    bool_t bIsFapaActive = ME_FALSE;
    float32_t fX;
    float32_t fY;
    float32_t fRangeFlankRightY;
    float32_t fRangeFlankLeftY;
    float32_t fRangeFlankFarRightY;
    float32_t fRangeFlankFarLeftY;
    float32_t fRangeFlankMinX;
    float32_t fRangeTubeMinX;
    float32_t fRangeTubeMaxX;
    float32_t fRangeFlankMaxX;
    float32_t fDistTraveled;
    float32_t fCloseThreshXRight = 1200.0f;
    float32_t fCloseThreshXLeft  = 1200.0f;
    float32_t fCloseThreshXTube  = 1200.0f;
    float32_t fThresholdDist = 0.0f;

    US_E_App_Prndl_Status_e ePrndl = UssCom_F_GetVehPrndl();
    US_S_DrivingTubeInfo_t * psDrivingTubeInfo = UssOD_psGetDrivingTubeInfo();
    uint8_t * puGrpSnrPntSortIdx = uSnrPntSortIdx[uGrpIdx];
    US_S_SnrPoint_t * psSnrPntGrp = sSnrPnt[uGrpIdx];
    US_S_SnrPoint_t * psSnrPnt;


    if (ePrndl != ePrndlPrev) /* On gear change */
    {
        uFlankFSMState = US_D_OD_FLANK_FSM_IDLE;  // Switching drive direction, so reset FSM.
    }

    if (   ((uGrpIdx == US_D_PHYS_GRP_FRNT) && (ePrndl == US_PRNDL_DRIVE))
        || ((uGrpIdx == US_D_PHYS_GRP_REAR) && (ePrndl == US_PRNDL_REVERSE)))
    {
        bUseThisBus = ME_TRUE;
    }

    if (bUseThisBus == (bool_t)ME_TRUE)
    {
#if (US_D_USE_FAPA_API == ME_TRUE)
        TbAP_DriveAssistStatOut_t sDriveAssistStatOut;
        SigMgr_PpDriveAssistStatOut_TbAP_DriveAssistStatOut_t_Get(&sDriveAssistStatOut);
        /* Do not perform mitigation in FAPA mode. */
        bIsFapaActive = (TeAutoPark_e_AutoParkStatus_Active == sDriveAssistStatOut.IeAP_e_AutoParkStatus)
                        && (TeAP_e_AlgoState_Park_In == sDriveAssistStatOut.IeAP_e_AlgoState);
#endif
        if (ME_FALSE == bIsFapaActive)
        {
            fRangeFlankLeftY     = psDrivingTubeInfo->fTubeEdgeY + US_D_OD_VEH_WIDTH_OFFSET;    // Inner +Y edge of flank
            fRangeFlankRightY    = -fRangeFlankLeftY;                                            // Inner -Y edge of flank
            fRangeFlankFarRightY = fRangeFlankRightY - US_D_OD_TUBE_CHK_FAR_Y_OFFSET; // Outer -Y edge of flank
            fRangeFlankFarLeftY  = fRangeFlankLeftY  + US_D_OD_TUBE_CHK_FAR_Y_OFFSET; // Outer +Y edge of flank

            if (uGrpIdx == US_D_PHYS_GRP_FRNT)
            {
                fRangeFlankMinX = psDrivingTubeInfo->fFrontBumperX + US_D_OD_TUBE_CHK_FLANK_MIN_X_OFFSET;   // Closest to ego, X edge of Flank area
                fRangeTubeMinX  = psDrivingTubeInfo->fFrontBumperX + US_D_OD_TUBE_CHK_TUBE_MIN_X_OFFSET;    // Closest to ego, X edge of Tube area
                fRangeTubeMaxX  = psDrivingTubeInfo->fFrontBumperX + US_D_OD_TUBE_CHK_MAX_X_OFFSET;         // Farthest from ego, max X Tube area
                fRangeFlankMaxX = psDrivingTubeInfo->fFrontBumperX + US_D_OD_TUBE_CHK_FLANK_MAX_X_OFFSET;   // Farthest from ego, max x Flank areas
            }
            else
            {
                // Note: These are converted to positive values on purpose.  Will be compared with |x| coord.
                fRangeFlankMinX = -psDrivingTubeInfo->fRearBumperX + US_D_OD_TUBE_CHK_FLANK_MIN_X_OFFSET;   // Closest to ego, X edge of Flank area
                fRangeTubeMinX  = -psDrivingTubeInfo->fRearBumperX + US_D_OD_TUBE_CHK_TUBE_MIN_X_OFFSET;    // Closest to ego, X edge of Tube area
                fRangeTubeMaxX  = -psDrivingTubeInfo->fRearBumperX + US_D_OD_TUBE_CHK_MAX_X_OFFSET;         // Farthest from ego, max X Tube area
                fRangeFlankMaxX = -psDrivingTubeInfo->fRearBumperX + US_D_OD_TUBE_CHK_FLANK_MAX_X_OFFSET;   // Farthest from ego, max x Flank areas
            }


            /* Check for flanking points, and find out how close they are in the|x| direction */
            for (uListIdx = ZERO; uListIdx < uNumCurPnts; uListIdx++)
            {
                /* Get index and pointer to point data in sorted order */
                uPntIdx = puGrpSnrPntSortIdx[uListIdx];
                psSnrPnt = &psSnrPntGrp[uPntIdx];

                uMinConfToQualify = US_D_OF_FLANK_MIN_CONF_TRIANG;

                /* Check if we need to raise confidence of non-side, far, non-triang points */
                if (psSnrPnt->uLowDirDist >= US_D_OF_FLANK_MIN_CONF_DIST_THRESH)  // Point is beyond dist thresh
                {
                    uDirSnrDataIdx = psSnrPnt->uDirSnrIdx;
                        bIsSideSensor = (bool_t) (   (uDirSnrDataIdx == US_D_SENSOR_FSR)
                                                  || (uDirSnrDataIdx == US_D_SENSOR_FSL)
                                                  || (uDirSnrDataIdx == US_D_SENSOR_RSR)
                                                  || (uDirSnrDataIdx == US_D_SENSOR_RSL));

                    if (   (ME_FALSE == bIsSideSensor)  // Not a side sensor
                        && ((psSnrPnt->uPntStat & US_D_PNTSTAT_TRIANG_LATCHED_BITS) == ZERO)) // Not triangulated by DI or DD
                    {
                        uMinConfToQualify = US_D_OF_FLANK_MIN_CONF_NON_TRIANG;
                    }
                }

                /* Scan for flanking points */
                if (psSnrPnt->uConf > uMinConfToQualify)  // Check if valid point
                {
                    fX = psSnrPnt->fX;
                    fY = psSnrPnt->fY;

                    /* Take absolute value of X sensor coord */
                    if (fX < 0.0f)
                    {
                        fX = -fX;
                    }

                    if ((fX >= fRangeFlankMinX) && (fX <= fRangeFlankMaxX))
                    {
                        if ((fY > fRangeFlankFarRightY) && (fY < fRangeFlankRightY))
                        {
                            bIsFlankedRight = ME_TRUE;
                            if (fX < fCloseThreshXRight)
                            {
                                fCloseThreshXRight = fX; // Store lowest |x| for flank.
                            }
                        }
                        else if ((fY < fRangeFlankFarLeftY) && (fY > fRangeFlankLeftY))
                        {
                            bIsFlankedLeft  = ME_TRUE;
                            if (fX < fCloseThreshXLeft)
                            {
                                fCloseThreshXLeft = fX; // Store lowest |x| for flank.
                            }
                        }
                        else if ((fY > -psDrivingTubeInfo->fTubeEdgeY) && (fY < psDrivingTubeInfo->fTubeEdgeY))
                        {
                            /*
                             * We are in the tube.  Need to see if this area is mostly clear.
                             * Find fresh and mature points in the middle.
                             */
                            if (   (psSnrPnt->uCruisingAge < 10u)
                                && (psSnrPnt->uAge >= 10u)
                                && ((psSnrPnt->uPntStat & US_D_PNTSTAT_DIR_IND_TRIANG_LTCH) != ZERO))  /* DI triangulation */
                            {
                                uTubePntCnt++;
                                if (fX < fCloseThreshXTube)
                                {
                                    fCloseThreshXTube = fX; // Store lowest |x| for flank.
                                }
                            }
                        }
                        else
                        {
                            /* Nothing to do.  Points outside far enough to ignore. */
                        }
                    }
                }
            }

            /* Reset this distance as there wasn't enough points in tube to call it a wall. */
            if (uTubePntCnt < THREE)
            {
                fCloseThreshXTube  = 1200.0f;
            }

            /*
             * FSM: IDLE -> CONFIRM -> ACTIVE -> INHIBIT -> IDLE
             *
             * At any time direction change resets FSM -> IDLE
             *
             */
            switch (uFlankFSMState)
            {
                case US_D_OD_FLANK_FSM_IDLE:
                {
                    if ((bIsFlankedRight == (bool_t)ME_TRUE) && (bIsFlankedLeft == (bool_t)ME_TRUE))  // Points are flanked
                    {

                        fThresholdDist = (fCloseThreshXLeft < fCloseThreshXRight) ? (fCloseThreshXLeft - fRangeFlankMinX) : (fCloseThreshXRight - fRangeFlankMinX);

                        if (fThresholdDist < 0.0f)
                        {
                            fThresholdDist = 0.0f;
                        }

                        fCapConfActiveDist = fThresholdDist + US_D_OD_FLANK_ACTIVE_DIST;

                        fMotionTrkX = 0.0f; // Reset motion tracker at initial trigger
                        fMotionTrkY = 0.0f; // Reset motion tracker at initial trigger

                        uFlankConfirmCnt = TWO;

                        uFlankFSMState = US_D_OD_FLANK_FSM_CONFIRM;
                    }

                }
                break;

                case US_D_OD_FLANK_FSM_CONFIRM:
                {
                    /* Checking for left and right flanking points plus an indentation
                     * to discriminate with wall.
                     */
                    if (   ((bool_t)ME_TRUE == bIsFlankedRight)
                        && ((bool_t)ME_TRUE == bIsFlankedLeft)
                        && (fCloseThreshXTube > (fThresholdDist + 30.0f)))
                    {
                        uFlankConfirmCnt += TWO;

                        if (uFlankConfirmCnt >= SIX)
                        {
                            bFirstActiveCycle = ME_TRUE;
                            uFlankFSMState = US_D_OD_FLANK_FSM_ACTIVE;
                        }
                    }
                    else
                    {
                        if (uFlankConfirmCnt > ZERO)
                        {
                            uFlankConfirmCnt--;
                        }
                        else
                        {
                            uFlankFSMState = US_D_OD_FLANK_FSM_IDLE;
                        }
                    }
                }
                break;

                case US_D_OD_FLANK_FSM_ACTIVE:
                {
                    fDistTraveled = (float32_t)sqrtf(fMotionTrkX * fMotionTrkX + fMotionTrkY * fMotionTrkY);
                    if (fDistTraveled > fCapConfActiveDist)
                    {
                        uFlankFSMState = US_D_OD_FLANK_FSM_INHIBIT;
                    }
                }
                break;

                case US_D_OD_FLANK_FSM_INHIBIT:
                {
                    fDistTraveled = (float32_t)sqrtf(fMotionTrkX * fMotionTrkX + fMotionTrkY * fMotionTrkY);

                    if (fDistTraveled > (fCapConfActiveDist + US_D_OD_FLANK_INHIBIT_DIST))
                    {
                        uFlankFSMState = US_D_OD_FLANK_FSM_IDLE;
                    }
                }
                break;

                default:
                {
                    uFlankFSMState = US_D_OD_FLANK_FSM_IDLE;
                }
                break;
            }

            /*
             * Cap confidence on points when FSM state for countermeasure is active
             *
             */
            if (US_D_OD_FLANK_FSM_ACTIVE == uFlankFSMState)
            {
                /* Cap confidence in existing point list */
                for (uListIdx = ZERO; uListIdx < uNumCurPnts; uListIdx++)
                {
                    /* Get index and pointer to point data in sorted order */
                    uPntIdx = puGrpSnrPntSortIdx[uListIdx];
                    psSnrPnt = &psSnrPntGrp[uPntIdx];

                    if (psSnrPnt->uConf != ZERO)  // Check if valid point
                    {
                        fX = psSnrPnt->fX;
                        fY = psSnrPnt->fY;

                        /* Take absolute value of X sensor coord */
                        if (fX < 0.0f)
                        {
                            fX = -fX;
                        }

                        if ((fX >= fRangeTubeMinX) && (fX <= fRangeTubeMaxX))
                        {
                            /* Possibly make an exception of point was DI triang
                             * and strictly within the primary driving tube.
                             */
                            bool_t bIsException =  (bool_t) (   (psSnrPnt->uCruisingAge < 10u)
                                                             && (psSnrPnt->uAge >= 10u)
                                                             && ((psSnrPnt->uPntStat & US_D_PNTSTAT_DIR_IND_TRIANG_LTCH) != ZERO)  /* DI triangulation */
                                                             && (fY > -psDrivingTubeInfo->fTubeEdgeY)    /* And in driving tube */
                                                             && (fY <  psDrivingTubeInfo->fTubeEdgeY));

                            /* Cap if first active cycle or no exception */
                            if ((ME_TRUE == bFirstActiveCycle) || (ME_FALSE == bIsException))
                            {
                                if (psSnrPnt->uConf > US_D_OD_TUBE_CHK_CONF_CAP)
                                {
                                    psSnrPnt->uConf = US_D_OD_TUBE_CHK_CONF_CAP;
                                }
                            }
                        }
                    }
                }

                bFirstActiveCycle = ME_FALSE; /* We are done with exceptions forcing first cycle cap. */

                /* Cap confidence in new point list */
                for (uPntIdx = ZERO; uPntIdx < uNumNewPnts; uPntIdx++)
                {
                    if (sSnrNewPntBuff[uPntIdx].uConf != ZERO) // Check if valid point
                    {
                        fX = sSnrNewPntBuff[uPntIdx].fX;
                        fY = sSnrNewPntBuff[uPntIdx].fY;

                        /* Take absolute value of X sensor coordinate */
                        if (fX < 0.0f)
                        {
                            fX = -fX;
                        }

                        if ((fX >= fRangeTubeMinX) && (fX <= fRangeTubeMaxX))
                        {
                            if ((fY >= fRangeFlankRightY) && (fY <= fRangeFlankLeftY))
                            {
                                /* Cap always cap new points for first cycle.
                                 * No exceptions here because they do not meet the
                                 * min age requirement.
                                 */
                                if (sSnrNewPntBuff[uPntIdx].uConf > US_D_OD_TUBE_CHK_CONF_CAP)
                                {
                                    sSnrNewPntBuff[uPntIdx].uConf = US_D_OD_TUBE_CHK_CONF_CAP;
                                }
                            }
                        }
                    }
                }
            }
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)
            uDbgCapConfFlag = uFlankFSMState + ONE; // FAPA Blocking is Zero.
#endif
        }
#if (US_D_USE_FAPA_API == ME_TRUE)
        else
        {
            uFlankFSMState = US_D_OD_FLANK_FSM_IDLE;  // Switching drive direction, so reset FSM.
#if (ME_TRUE == US_D_OD_DEBUG_VARS_ON)       
            uDbgCapConfFlag = ZERO;
#endif
        }
#endif
    }
}

static bool_t UssOD_bIsFlankingMitigationActive(void)
{
    return ((bool_t) (US_D_OD_FLANK_FSM_ACTIVE == uFlankFSMState));
}

#endif

#if (US_D_OD_KEEP_DEAD_PTS == ME_TRUE)

/* Dead point buffer - add to ring buffer */
static void UssOD_AddPntToDeadPntBuff(uint8_t uGrpIdx, US_S_SnrPoint_t * psSnrPnt)
{
    psSnrPnt->uConf = psSnrPnt->uMaxConf;      // Restore max conf the point ever had
    psSnrPnt->uDeadPntStat |= US_D_DPS_KEPT;   // Set dead point stat as kept for possible resurrection later.

    /* Store point in left or right dead point ring buffer. */
    if (psSnrPnt->fY > 0.0f) // Left Side
    {
        /* Advance ring head pointer */
        uDeadPntRingLeftHead[uGrpIdx]++;
        if (uDeadPntRingLeftHead[uGrpIdx] >= US_D_DEAD_PNT_BUFF_SIZE)
        {
            uDeadPntRingLeftHead[uGrpIdx] = ZERO;
        }

        /* If head reaches tail, then overwrite last point in ring */
        if (uDeadPntRingLeftHead[uGrpIdx] == uDeadPntRingLeftTail[uGrpIdx])
        {
            uDeadPntRingLeftTail[uGrpIdx]++;
            if (uDeadPntRingLeftTail[uGrpIdx] >= US_D_DEAD_PNT_BUFF_SIZE)
            {
                uDeadPntRingLeftTail[uGrpIdx] = ZERO;
            }
        }

        /* Store point at head location */
        memcpy(&sSnrDeadPntLeftBuff[uGrpIdx][uDeadPntRingLeftHead[uGrpIdx]], psSnrPnt, sizeof(US_S_SnrPoint_t));
    }
    else // Right Side
    {
        /* Advance ring head pointer */
        uDeadPntRingRightHead[uGrpIdx]++;
        if (uDeadPntRingRightHead[uGrpIdx] >= US_D_DEAD_PNT_BUFF_SIZE)
        {
            uDeadPntRingRightHead[uGrpIdx] = ZERO;
        }

        /* If head reaches tail, then overwrite last point in ring */
        if (uDeadPntRingRightHead[uGrpIdx] == uDeadPntRingRightTail[uGrpIdx])
        {
            uDeadPntRingRightTail[uGrpIdx]++;
            if (uDeadPntRingRightTail[uGrpIdx] >= US_D_DEAD_PNT_BUFF_SIZE)
            {
                uDeadPntRingRightTail[uGrpIdx] = ZERO;
            }
        }

        /* Stor point at head location */
        memcpy(&sSnrDeadPntRightBuff[uGrpIdx][uDeadPntRingRightHead[uGrpIdx]], psSnrPnt, sizeof(US_S_SnrPoint_t));
    }

}

static void UssOD_UpdateOdometryForDeadPnts(uint8_t uGrpIdx,
                                            float32_t fCos,
                                            float32_t fSin,
                                            float32_t fDelX,
                                            float32_t fDelY)
{
    US_S_SnrPoint_t *psSnrDeadPnt;
    float32_t fX;
    float32_t fY;
    float32_t fRotX;
    float32_t fRotY;
    uint8_t uTail;

    /* Odometry for right dead point buf */
    uTail = uDeadPntRingRightTail[uGrpIdx];
    while (uTail != uDeadPntRingRightHead[uGrpIdx])
    {
        psSnrDeadPnt = &sSnrDeadPntRightBuff[uGrpIdx][uTail];
        if (psSnrDeadPnt->uConf > ZERO)
        {
            fX =  psSnrDeadPnt->fX;
            fY =  psSnrDeadPnt->fY;

            fRotX = fX * fCos - fY * fSin;
            fRotY = fX * fSin + fY * fCos;

            psSnrDeadPnt->fX = fRotX - fDelX;
            psSnrDeadPnt->fY = fRotY - fDelY;

            if (   (psSnrDeadPnt->fX > US_D_OD_KDP_DEL_DIST_CM_FRNT)
                || (psSnrDeadPnt->fX < US_D_OD_KDP_DEL_DIST_CM_REAR) )
            {
                psSnrDeadPnt->uDeadPntStat |= US_D_DPS_DIST_DEL; /* mark it for debug */
                psSnrDeadPnt->uConf = ZERO; /* Delete the point completely */
            }
        }

        uTail++;
        if (uTail >= US_D_DEAD_PNT_BUFF_SIZE)
        {
            uTail = ZERO;
        }
    }

    /* Odometry for left dead point buff */
    uTail = uDeadPntRingLeftTail[uGrpIdx];
    while (uTail != uDeadPntRingLeftHead[uGrpIdx])
    {
        psSnrDeadPnt = &sSnrDeadPntLeftBuff[uGrpIdx][uTail];
        if (psSnrDeadPnt->uConf > ZERO)
        {

            fX =  psSnrDeadPnt->fX;
            fY =  psSnrDeadPnt->fY;

            fRotX = fX * fCos - fY * fSin;
            fRotY = fX * fSin + fY * fCos;

            psSnrDeadPnt->fX = fRotX - fDelX;
            psSnrDeadPnt->fY = fRotY - fDelY;

            if (   (psSnrDeadPnt->fX > US_D_OD_KDP_DEL_DIST_CM_FRNT)
                || (psSnrDeadPnt->fX < US_D_OD_KDP_DEL_DIST_CM_REAR) )
            {
                psSnrDeadPnt->uDeadPntStat |= US_D_DPS_DIST_DEL; /* mark it for debug */
                psSnrDeadPnt->uConf = ZERO; /* Delete the point completely */
            }

            uTail++;
            if (uTail >= US_D_DEAD_PNT_BUFF_SIZE)
            {
                uTail = ZERO;
            }
        }
    }
}

static void UssOD_ResurrectDeadPnts(uint8_t uGrpIdx, uint8_t uTmpLeftOrRight, float32_t fAvgY) // 1 for right for now
{
    US_S_SnrPoint_t *psSnrNewPnt;
    US_S_SnrPoint_t *psSnrDeadPnt;
    uint8_t uDeadPntIdx;
    uint8_t uMatchPntIdx;

    /* Puts as many dead points as it can, into new point list */
    if (uTmpLeftOrRight) // Assume right for now
    {
        /* Adding right side points - newest dead first (LIFO RING) */
        uDeadPntIdx = uDeadPntRingRightHead[uGrpIdx];
        while (uDeadPntIdx != uDeadPntRingRightTail[uGrpIdx])
        {
            /* Is there space for the points in the new point list? */
            if (uNumNewPnts < US_D_POINT_BUFFER_SIZE)
            {
                psSnrNewPnt  = &sSnrNewPntBuff[uNumNewPnts];
                psSnrDeadPnt = &sSnrDeadPntRightBuff[uGrpIdx][uDeadPntIdx];

                /* Does point qualify to be added? */
#if 0
                if (   (psSnrDeadPnt->fY < (fAvgY + 100.0f))
                    && (psSnrDeadPnt->fY > (fAvgY - 100.0f))
                    && (psSnrDeadPnt->uConf > ZERO))
#else
                  if (psSnrDeadPnt->uConf > ZERO)
#endif
                {
                    uMatchPntIdx = UssOD_MatchPointWithPntList(uGrpIdx, psSnrDeadPnt->fX, psSnrDeadPnt->fY, ME_FALSE);
                    if (uMatchPntIdx == US_D_OD_NO_MATCH_FOUND)
                    {
                        memcpy(psSnrNewPnt, psSnrDeadPnt, sizeof(US_S_SnrPoint_t));
                    }
                    else // Just top off existing point if lower conf.
                    {
                        if (sSnrPnt[uGrpIdx][uMatchPntIdx].uConf < psSnrDeadPnt->uConf)
                        {
                            sSnrPnt[uGrpIdx][uMatchPntIdx].uConf = psSnrDeadPnt->uConf;
                        }
                    }
                    /* Initialize sort index for this cycle */
                    uSnrNewPntSortIdx[uNumNewPnts] = uNumNewPnts;
                    uNumNewPnts++;
                }

                uDeadPntIdx--;  // Decrement to previous dead point
                if (0xFF == uDeadPntIdx)
                {
                    uDeadPntIdx = US_D_DEAD_PNT_BUFF_SIZE - ONE;
                }
            }
            else // We are out of room in the new point buffer.  The rest of the points will remain dead.
            {
                break;
            }
        }
    }
    else
    {
        /* Adding left side points - newest dead first (LIFO RING) */
        uDeadPntIdx = uDeadPntRingLeftHead[uGrpIdx];
        while (uDeadPntIdx != uDeadPntRingLeftTail[uGrpIdx])
        {
            if (uNumNewPnts < US_D_POINT_BUFFER_SIZE)
            {
                psSnrNewPnt  = &sSnrNewPntBuff[uNumNewPnts];
                psSnrDeadPnt = &sSnrDeadPntLeftBuff[uGrpIdx][uDeadPntRingLeftTail[uGrpIdx]];

                /* Does point qualify to be added? */
#if 0
                if (   (psSnrDeadPnt->fY < (fAvgY + 100.0f))
                    && (psSnrDeadPnt->fY > (fAvgY - 100.0f))
                    && (psSnrDeadPnt->uConf > ZERO))
#else
                  if (psSnrDeadPnt->uConf > ZERO)
#endif
                {

                    /*
                     * CHECK FOR MATCH WITH *EXISTING POINTS*
                     */
                    uMatchPntIdx = UssOD_MatchPointWithPntList(uGrpIdx, psSnrDeadPnt->fX, psSnrDeadPnt->fY, ME_FALSE);
                    if (uMatchPntIdx == US_D_OD_NO_MATCH_FOUND)
                    {
                        memcpy(psSnrNewPnt, psSnrDeadPnt, sizeof(US_S_SnrPoint_t));
                    }
                    else // Just top off existing point if lower conf.
                    {
                        if (sSnrPnt[uGrpIdx][uMatchPntIdx].uConf < psSnrDeadPnt->uConf)
                        {
                            sSnrPnt[uGrpIdx][uMatchPntIdx].uConf = psSnrDeadPnt->uConf;
                        }
                    }

                    /* Initialize sort index for this cycle */
                    uSnrNewPntSortIdx[uNumNewPnts] = uNumNewPnts;
                    uNumNewPnts++;
                }

                uDeadPntIdx--;   // Advance to previous dead point
                if (0xFF == uDeadPntIdx)
                {
                    uDeadPntIdx = US_D_DEAD_PNT_BUFF_SIZE - ONE;
                }
            }
            else  // We are out of room in the new point buffer.  The rest of the points will remain dead.
            {
                break;
            }
        }
    }
    _dbg_NecPnt_nNewPnt = uNumNewPnts;
}

/* Check if we are entering into FAPA to ressurect dead points */
static void UssOD_CheckForDeadPntResurrection(uint8_t uGrpIdx)
{
    float32_t fTotalY;
    float32_t fAvgY;
    bool_t bIsFapaParkInActive = ME_FALSE;
    uint8_t uTail;
    uint8_t uNumPntsToAvg;

#if (US_D_USE_FAPA_API == ME_TRUE)
    TbAP_DriveAssistStatOut_t sDriveAssistStatOut;
    SigMgr_PpDriveAssistStatOut_TbAP_DriveAssistStatOut_t_Get(&sDriveAssistStatOut);
    /* Do not perform mitigation in FAPA mode. */
    bIsFapaActive = (TeAutoPark_e_AutoParkStatus_Active == sDriveAssistStatOut.IeAP_e_AutoParkStatus)
                    && (TeAP_e_AlgoState_Park_In == sDriveAssistStatOut.IeAP_e_AlgoState);

    _dbg_NecPnt_FapaMode = 0u;
    if (TeAutoPark_e_AutoParkStatus_Active == sDriveAssistStatOut.IeAP_e_AutoParkStatus)
        _dbg_NecPnt_FapaMode |= 0x01;

    if  (TeAP_e_AlgoState_Park_In == sDriveAssistStatOut.IeAP_e_AlgoState)
        _dbg_NecPnt_FapaMode |= 0x02;

    if (ME_TRUE == bIsFapaActive)
        _dbg_NecPnt_FapaMode |= 0x04;

#endif

    /* Check if entering FAPA Park In manuver*/
    if (  (bIsFapaParkInActivePrev != bIsFapaParkInActive)
        && (ME_TRUE == bIsFapaParkInActive))
    {
        uNumPntsToAvg = ZERO;
        fTotalY = 0.0f;

        if (1) // Right side, placeholder for actual API
        {
            /* Analyze right side points */
            uTail = uDeadPntRingRightTail[uGrpIdx];
            while (uTail != uDeadPntRingRightHead[uGrpIdx])
            {
                fTotalY +=  sSnrDeadPntRightBuff[uGrpIdx][uTail].fY;
                uNumPntsToAvg++;

                uTail++;
                if (uTail >= US_D_DEAD_PNT_BUFF_SIZE)
                {
                    uTail = ZERO;
                }
            }
        }
        else
        {
            /* Analyze left side points */
            uNumPntsToAvg = ZERO;
            uTail = uDeadPntRingLeftTail[uGrpIdx];

            while (uTail != uDeadPntRingLeftHead[uGrpIdx])
            {
                fTotalY +=  sSnrDeadPntLeftBuff[uGrpIdx][uTail].fY;
                uNumPntsToAvg++;

                uTail++;
                if (uTail >= US_D_DEAD_PNT_BUFF_SIZE)
                {
                    uTail = ZERO;
                }
            }
        }


        if (uNumPntsToAvg > ZERO)
        {
            fAvgY = fTotalY / (float32_t) uNumPntsToAvg;
            UssOD_ResurrectDeadPnts(uGrpIdx, ONE, fAvgY);
        }

        bIsFapaParkInActivePrev = bIsFapaParkInActive;
    }
}

#endif

/*===========================================================================
 * @brief Create default sort index lists
 *
 * @param
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_CreateSortIdxLists
 * Remarks:  DD-ID: {33384300-8BC5-47cc-9E52-D37438027F77}
 * ===========================================================================*/
void UssOD_CreateSortIdxLists(void)
{
    uint8_t uGrpIdx;
    uint8_t uPntIdx;

    for (uGrpIdx = ZERO; uGrpIdx < US_D_PHYS_GRP_MAX; uGrpIdx++)
    {
        for (uPntIdx = ZERO; uPntIdx < US_D_POINT_BUFFER_SIZE; uPntIdx++)
        {
            uSnrPntSortIdx[uGrpIdx][uPntIdx] = uPntIdx;
        }

        /* Not part of list creation, other generic init for this file */
        uSuppressCounter[uGrpIdx] = ZERO;
        eGrpAllowLevel[uGrpIdx] = OD_GRP_ALL_ALLOW;
        g_prevGrpState[uGrpIdx] = SYSMGR_GRPSTATE_OK;
    }
}

/*===========================================================================
 * @brief Returns pointer to physical group point map.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_sGetSnrPntBuff
 * Remarks:  DD-ID: {0E999CD5-5F63-43e6-B7B1-FE7EC811B903}
 * Req.-ID: 13477648
 * ===========================================================================*/
US_S_SnrPoint_t * UssOD_sGetSnrPntBuff(uint8_t uGrpIdx)
{
    return sSnrPnt[uGrpIdx];
}

/*===========================================================================
 * @brief Returns pointer to sorted index map of physical group point map.
 *
 * @param uint8_t uGrpIdx
 * @param
 * @param
 * @return
 * @remarks
 */
/* ===========================================================================
 * Name:	 UssOD_sGetSnrPntSortIdxList
 * Remarks:  DD-ID: {DE77119E-0927-481d-B4A1-8F2B5546BB87}
 * ===========================================================================*/
uint8_t * UssOD_sGetSnrPntSortIdxList(uint8_t uGrpIdx)
{
    return uSnrPntSortIdx[uGrpIdx];
}


#ifdef US_D_OD_DEBUG_VARS_ON
/* Debug output, could be used in other OD too */
US_E_Grp_Allow_Level_t UssOD_eGetGrpAllowLevel(uint8_t uGrpIdx)
{
    return eGrpAllowLevel[uGrpIdx];
}
#endif

/*
 *  UNIT TEST FUNCTIONS
 */
#if (US_D_TD_UNIT_TESTING_MODE != ZERO)
#define US_D_TD_UNIT_TESTING_SUB_MODE (6)

void UssOD_TemporalDirUnitTest(uint8_t uDirSnrDataIdx,
                               bool_t bIsRearGroup,
                               uint16_t *uDirDist,
                               uint16_t *uIndirDist,
                               float32_t fDeltaX,
                               const US_S_Sensor_coordinate_t * pSnrCoord,
                               US_S_EchoCacheEntry_t *psEchoCacheEntry)
{
#if (US_D_TD_UNIT_TESTING_SUB_MODE == 0)
      (void) uDirSnrDataIdx; (void) bIsRearGroup; (void) pSnrCoord; (void) psEchoCacheEntry;
      *uDirDist = 200;  // wings
      *uIndirDist = *uDirDist; // Side checking

#elif (US_D_TD_UNIT_TESTING_SUB_MODE == 1)
    (void) uDirSnrDataIdx; (void) pSnrCoord; (void) psEchoCacheEntry;
    float32_t fTestDelta;
    if (ME_TRUE == bIsRearGroup) // Center steady
      {
         fTestDelta = fDeltaX; // reverse, rear, default pnts,, works perfectly.
         *uDirDist = 200;
         *uIndirDist = (float32_t) *uDirDist + fTestDelta; // Side checking - line in center in back, in forward wing in back
      }
      else
      {
         fTestDelta = -fDeltaX; // reverse, front, default pnts,, works perfectly.  -wing in front, in forward wing in front
         *uDirDist = 200;
         *uIndirDist = (float32_t) *uDirDist + fTestDelta; // Side checking
    }

#elif (US_D_TD_UNIT_TESTING_SUB_MODE == 2)
    (void) uDirSnrDataIdx; (void) pSnrCoord; (void) psEchoCacheEntry;
    float32_t fTestDelta;
    if (ME_TRUE == bIsRearGroup) // Center steady  45 deg
    {
        fTestDelta = fDeltaX; // reverse, rear, default pnts,, works perfectly.
        *uDirDist = 200;
        *uIndirDist = (float32_t) *uDirDist + fTestDelta * 0.707f; // Side checking - line in center in back, in forward wing in back
    }
    else
    {
        fTestDelta = -fDeltaX; // reverse, front, default pnts,, works perfectly.  -wing in front, in forward wing in front
        *uDirDist = 200;
        *uIndirDist = (float32_t) *uDirDist + fTestDelta * 0.707f; // Side checking
    }

#elif (US_D_TD_UNIT_TESTING_SUB_MODE == 3)
    (void) uDirSnrDataIdx; (void) bIsRearGroup; (void) pSnrCoord; (void) psEchoCacheEntry;
    // distance check
    float32_t fTestDelta;
    static float32_t cosang = 200.0f;
    static float32_t dir = 0.3f;

    cosang += dir;
    if (((dir > 0) && (cosang > 350)) || ((dir < 0) && (cosang < 50)))
    {
      dir = -dir;
      cosang += dir;
    }

    if (ME_TRUE == bIsRearGroup) // Center steady  45 deg
    {
     fTestDelta = fDeltaX; // reverse, rear, default pnts,, works perfectly.
     *uDirDist = cosang;
     *uIndirDist = (float32_t) *uDirDist + fTestDelta * 0.707f; // Side checking - line in center in back, in forward wing in back
    }
    else
    {
     fTestDelta = -fDeltaX; // reverse, front, default pnts,, works perfectly.  -wing in front, in forward wing in front
     *uDirDist = cosang;
     *uIndirDist = (float32_t) *uDirDist + fTestDelta * 0.707f; // 45 deg
    }

#elif (US_D_TD_UNIT_TESTING_SUB_MODE == 4)
    // angle check
    static float32_t cosang = 1.0f;
    static float32_t dir = -0.001f;
    static uint16_t  uDwellStart = 400;
    static uint16_t dwell = 400;
    static uint16_t radius =  200;
    float32_t fTestDelta;


    cosang += dir;
    if (dwell != 0)
    {
      dwell--;
      if (dir < 0) cosang = 1.0f;
      else cosang = -0.9f;
    }
    else if (((dir > 0) && (cosang > 1.0f)) || ((dir < 0) && (cosang < -1.0f)))
    {
      dir = -dir;
      cosang += dir;
      dwell =  uDwellStart;
    }

    if (ME_TRUE == bIsRearGroup) // Center steady  45 deg
    {
     fTestDelta = fDeltaX; // reverse, rear, default pnts,, works perfectly.
     *uDirDist = radius;
     *uIndirDist = (float32_t) *uDirDist + fTestDelta * cosang; // Side checking - line in center in back, in forward wing in back
    }
    else
    {
     fTestDelta = -fDeltaX; // reverse, front, default pnts,, works perfectly.  -wing in front, in forward wing in front
     *uDirDist = radius;
     *uIndirDist = (float32_t) *uDirDist + fTestDelta * cosang; // Side checking
    }
    /* ********* END: DO NOT MODIFY ********/
#elif (US_D_TD_UNIT_TESTING_SUB_MODE == 5)

    float32_t fTargetYOffset;
    float32_t fDx, fDy;
    static float32_t fTargetX;
    static float32_t fTargetY;
    static float32_t fDelX = 0.0f;
    static float32_t fDelY = 0.0f;
    static float32_t fDirX = 0.4f;
    static float32_t fDirY = 0.2f;
    static uint16_t  uDwellStart = 400;
    static uint16_t  uDwellX = 400;
    static uint16_t  uDwellY = 400;
    bool_t bIsLeftIndir = (bool_t) (pSnrCoord[uDirSnrDataIdx].fY >= 0.0f); // Are we on the left side of the vehicle?

    fDelY += fDirY;

    if ( uDwellY != 0)
    {
       uDwellY--;
      //fDelY = 0.0f;
    }
    if (((fDirY > 0.0f) && (fDelY > 200.0f)) || ((fDirY < 0.0f) && (fDelY < 0.0f)))
    {
      fDirY = -fDirY;
      fDelY += fDirY;
       uDwellY =  uDwellStart;
    }

    fDelX += fDirX;
    if ( uDwellX != 0)
    {
       uDwellX--;
    //  fDelX = 0.0f;
    }
    if (((fDirX > 0.0f) && (fDelX > 200.0f)) || ((fDirX < 0.0f) && (fDelX < -200.0f)))
    {
      fDirX = -fDirX;
      fDelX += fDirX;
       uDwellX =  uDwellStart;
    }

    if (bIsRearGroup)
    {
      fTargetX = -200.0f - fDelX;
      fTargetYOffset = pSnrCoord[US_D_SENSOR_RSL].fY;
    }
    else
    {
      fTargetX = 500.0f + fDelX;
      fTargetYOffset = pSnrCoord[US_D_SENSOR_FSL].fY;
    }

    if (bIsLeftIndir)
    {
      fTargetY = fTargetYOffset + fDelY;
    }
    else
    {
      fTargetYOffset = -fTargetYOffset;
      fTargetY = fTargetYOffset - fDelY;
    }

    //                  if (uDirSnrDataIdx == US_D_SENSOR_ROL)
    {
      fDx = fTargetX - pSnrCoord[uDirSnrDataIdx].fX;
      fDy = fTargetY - pSnrCoord[uDirSnrDataIdx].fY;
      *uDirDist = (float32_t)sqrtf(fDx * fDx + fDy * fDy); // Calculate distance between target and sensor.

      fDx = fTargetX - psEchoCacheEntry->fVirtualSnrPosX ;
      fDy = fTargetY - psEchoCacheEntry->fVirtualSnrPosY;
      *uIndirDist = (float32_t)sqrtf(fDx * fDx + fDy * fDy); // Calculate distance between target and sensor.
    }
#elif (US_D_TD_UNIT_TESTING_SUB_MODE == 6)
    (void) fDeltaX;

    bool_t bIsLeftIndir = (bool_t) (pSnrCoord[uDirSnrDataIdx].fY >= 0.0f); // Are we on the left side of the vehicle?
    float32_t fTargetYOffset;
    float32_t fDx, fDy;
    static float32_t fDelX = 0.0f;
    static float32_t fDelY = 0.0f;
    static float32_t fDirX = 0.4f;
    static float32_t fDirY = 0.2f;
    static uint16_t  uDwellStart = 400;
    static uint16_t  uDwellX = 400;
    static uint16_t  uDwellY = 400;
    static float32_t fTargetX;
    static float32_t fTargetY;
    //if (bIsSideSensor) return ME_FALSE; // REMOVE - TEST CODE!!!

    fDelY += fDirY;

    if ( uDwellY != 0)
    {
       uDwellY--;
    }
    //if (((fDirY > 0.0f) && (fDelY > 95.0f)) || ((fDirY < 0.0f) && (fDelY < 0.0f)))
    if (((fDirY > 0.0f) && (fDelY > 200.0f)) || ((fDirY < 0.0f) && (fDelY < 0.0f)))
    {
      fDirY = -fDirY;
      fDelY += fDirY;
       uDwellY =  uDwellStart;
    }

    fDelX += fDirX;
    if ( uDwellX != 0)
    {
       uDwellX--;
    }
    if (((fDirX > 0.0f) && (fDelX > 200.0f)) || ((fDirX < 0.0f) && (fDelX < -200.0f)))
    {
      fDirX = -fDirX;
      fDelX += fDirX;
      uDwellX =  uDwellStart;
    }

    if (bIsRearGroup == ME_TRUE)
    {
      fTargetX = -290.0f - fDelX;
      fTargetYOffset = 0;//pSnrCoord[US_D_SENSOR_RIL].fY;
    }
    else
    {
      fTargetX = 600.0f + fDelX;
      fTargetYOffset = 0; //pSnrCoord[US_D_SENSOR_FIL].fY;
    }

    if (bIsLeftIndir)
    {
      fTargetY = fTargetYOffset + fDelY;
    }
    else
    {
      fTargetYOffset = -fTargetYOffset;
      fTargetY = fTargetYOffset - fDelY;
    }

//    fTargetX = 420;
//    if (bIsLeftIndir)
//    {
//        fTargetY = 139;
//    }
//    else
//    {
//        fTargetY = -139;
//    }
    //fTargetY = -88.0f;
    fDx = fTargetX - pSnrCoord[uDirSnrDataIdx].fX;
    fDy = fTargetY - pSnrCoord[uDirSnrDataIdx].fY;
    *uDirDist = (float32_t)sqrtf(fDx * fDx + fDy * fDy); // Calculate distance between target and sensor.

    fDx = fTargetX - psEchoCacheEntry->fVirtualSnrPosX;
    fDy = fTargetY - psEchoCacheEntry->fVirtualSnrPosY;
    *uIndirDist = (float32_t)sqrtf(fDx * fDx + fDy * fDy); // Calculate distance between target and sensor.

#elif (US_D_TD_UNIT_TESTING_MODE == 2)

    float32_t fTestDelta;
    if (ME_TRUE == bIsRearGroup)
    {
       fTestDelta = fDeltaX; // reverse, rear.
    }
    else
    {
       fTestDelta = -fDeltaX; // reverse, front
    }
        *uIndirDist = psDirEchoCache->sEchoCacheEntry[uCacheIdx].uDirDist + (uint16_t) fTestDelta;
    }
#endif
}
#endif
