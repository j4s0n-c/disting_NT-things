
#include <math.h>
#include <new>
#include <distingnt/api.h>

#ifndef PI
    #define PI             3.14159265
#endif // ! PI
#ifndef PI_HALF
    #define PI_HALF     (PI/2.0)
#endif // ! PI_HALF

#define BASE_FREQ_HZ       261.6256f // f0 (base frequency) in Hz

#define VOLTAGE_PARAM_SCALING     1 // scaling (n) in parameter definition
#define VOLTAGE_SCALING          10 // scaling (10^n)

#define    MIN_PARAMETER_VAL     (-(int16_t)(0x7FFF))   // Parameters are int16?
#define    MAX_PARAMETER_VAL     ((int16_t)(0x7FFF))   // Parameters are int16?
#define    DEF_PARAMETER_VAL     ((int16_t)0)


#define TS_POLYGEN_VERTICES_MIN        3    // Min # vertices/sides in polygon
#define TS_POLYGEN_VERTICES_MAX        36     // Max #vertices/sides in polygon. Wanted 32 but for a nice 3 sides/Volt, we can do 33 I guess
#define TS_POLYGEN_VERTICES_DEF        3     // Default # vertices/sides in polygon
#define TS_POLYGEN_ANGLE_OFFSET_DEG_MIN        -180    // Initial rotation/angle offset min
#define TS_POLYGEN_ANGLE_OFFSET_DEG_MAX         180    // Initial rotation/angle offset max
#define TS_POLYGEN_ANGLE_OFFSET_DEG_DEF           0    // Initial rotation/angle offset default

#define BUFF_SIZE   ((TS_POLYGEN_VERTICES_MAX*2))

#define TS_CV_INPUT_RANGE_MIN            -12.0f    // Min Rack states should be 'allowed'
#define TS_CV_INPUT_RANGE_MAX             12.0f    // Max Rack states should be 'allowed'
#define TS_CV_INPUT_MIN_DEF                  0.0f    // Our CV Input min value (was -5V), now 0 so it will match MIDI controllers.
#define TS_CV_INPUT_MAX_DEF                 10.0f    // Our CV Input max value (was +5 V), now 10 so it will match MIDI controllers.
#define TS_CV_OUTPUT_MIN_DEF            -10.0f    // Our CV OUTPUT minimum
#define TS_CV_OUTPUT_MAX_DEF             10.0f    // Our CV OUTPUT maximum

#define TS_POLYGEN_AMPL_MIN            -10.0f
#define TS_POLYGEN_AMPL_MAX            10.0f
#define TS_POLYGEN_AMPL_DEF            5.0f

#define TS_POLGEN_ROT_DEG_MIN        -720.0f  // Rotation min (degrees). Why did we make 2 rotations? Don't remember now...
#define TS_POLGEN_ROT_DEG_MAX         720.0f     // Rotation max (degrees)
#define TS_POLGEN_ROT_DEG_DEF           0.0f  // Rotation def (degrees)

// Inner Radius ======================
#define TS_POLYGEN_INNER_RADIUS_MULT_MIN    -5.0f // -500%
#define TS_POLYGEN_INNER_RADIUS_MULT_MAX     5.0f // +500%
#define TS_POLYGEN_INNER_RADIUS_MULT_DEF     1.0f //  100%

#define TS_POLYGEN_INNER_OFFSET_DEG_MIN     -5.0f
#define TS_POLYGEN_INNER_OFFSET_DEG_MAX     5.0f
#define TS_POLYGEN_INNER_OFFSET_DEG_DEF     0.0f

#define TS_POLYGEN_BUFF_SIZE            1024

#define SINFUNC(x)                    sinf(x)
#define COSFUNC(x)                    cosf(x)

#define SQRTFUNC(x)                   sqrtf(x)

#define SGN(x)      ( (x < 0.0f) ? -1.0f : 1.0f )

#define DEBUG_POLY        0

#define TS_POLYGEN_IRADIUS_REL_2_MID_POINT        1 // Inner radius multiplier is multiplied by 0:Outer Amplitude, 1:Mid Point of line between corners
#define TS_POLYGEN_MOD_ENABLED                    0 // Add modulation items. Currently we don't add these, we ran out of panel space and decided not needed since users can technically do this with the outputs in another module.
                                                  // If turned back on, we have to add controls and such for these inputs/parameters.
#define TS_POLYGEN_TRIGGER_SYNC_EARLY            0 // (1) Trigger sync 1 dt before next cycle or (0) wait until we are actually starting the next cycle.


struct Vec {
    float x;
    float y;
    Vec(){return;}
    Vec(float _x, float _y){
        this->x = _x;
        this->y = _y;
    }
};

float clamp(float val, float min, float max){
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

// Given the int16 value that a parameter gives us, rescale it into the float min,max we actually want.
float scale(int16_t paramVal, int16_t inMin, int16_t inMax, float outMin, float outMax) {
    return static_cast<float>(paramVal) * (outMax - outMin) / static_cast<float>(inMax - inMin);
}
float scale(int16_t paramVal, float outMin, float outMax) {
    return scale(paramVal, MIN_PARAMETER_VAL, MAX_PARAMETER_VAL, outMin, outMax);
}
float scale(float paramVal, float inMin, float inMax, float outMin, float outMax){
    return static_cast<float>(paramVal) * (outMax - outMin) / static_cast<float>(inMax - inMin);
}
int16_t scale_i(float paramVal, float inMin, float inMax, int16_t outMin, int16_t outMax){
    return static_cast<int16_t>(scale(paramVal, inMin, inMax, static_cast<float>(outMin), static_cast<float>(outMax) ));
}
int16_t scale_i(float paramVal, float inMin, float inMax){
    return static_cast<int16_t>(scale(paramVal, inMin, inMax, static_cast<float>(MIN_PARAMETER_VAL), static_cast<float>(MAX_PARAMETER_VAL) ));
}




struct _polyGenAlgorithm : public _NT_algorithm
{
    //_polyGenAlgorithm( _polyGenAlgorithm_DTC* dtc_ ) : dtc( dtc_ ) {}
    _polyGenAlgorithm() {}
	~_polyGenAlgorithm() {}
	
	//float gain;
    
    //_polyGenAlgorithm_DTC* dtc;
    //_polyGenInputs* dtc;
    // Phase
    float phase = 0.0f;
    // Current Vertex
    int currVertexIx = 0;
    // Next Vertex
    int nextVertexIx = 1;
    // Main shape, number of sides/vertices
    uint8_t numVertices = TS_POLYGEN_VERTICES_DEF;
    float angleOffset_rad = 0.0f;
    float xAmpl = TS_POLYGEN_AMPL_DEF;
    float yAmpl = TS_POLYGEN_AMPL_DEF;
    float xOffset = 0.0f;
    float yOffset = 0.0f;
    // Pre offset (center of rotation) X
    float xCRot = 0.0f;
    // Pre offset (center of rotation) Y
    float yCRot = 0.0f;
    // Rotation
    bool rotationIsAbs = true;
    float rotation_rad = 0.0f;
    float rotation_deg = 0.0f;
    int lastRotationAbs = -1;

    // Frequency 
    float frequencyParam_V = 0.0f;
    
    //=== * Inner Vertices * ===
    float innerRadiusMult = TS_POLYGEN_INNER_RADIUS_MULT_DEF;     // Multiplier for radius (relative to main shape)
    float innerAngleMult = TS_POLYGEN_INNER_OFFSET_DEG_DEF;     // Multiplier for angle (relative to the mid-angle of main shape)
    float innerPhase = 0.0f;         // Inner/2ndary phase.
    int innerSideIx = 0;            // Which side we are on (from inner/2ndary point). Either 0 (before inner vertex) or 1 (after inner vertex).
    bool useInnerVerts = false;

    // UI
    bool topBarOn = true;
};

// Parameter ids/indices
enum ParamIds : uint8_t
{
    // Frequency Input (1V/Octave)
    kParamInput,
	kParamOutput,
	kParamOutputMode,
	kParamOutput2,
	kParamOutputMode2,
    // Frequency (Hz) - 1 cycle = 1 shape, so (shapes/s)
    FREQ_PARAM,
    // Number of outer vertices. 'Inner' vertices will be mapped in between, but by default will be in-line with the outer vertices.
    NUM_VERTICES_PARAM,
    // Angle offset for shape / Initial rotation
    ANGLE_OFFSET_PARAM,
    // Radius of inner vertices relative to the outer radius. Default is 1 (no star).
    INNER_VERTICES_RADIUS_PARAM,
    // Angle offset of the inner vertices. Default is 0 degrees from 180/N (mid).
    INNER_VERTICES_ANGLE_PARAM,
    X_AMPLITUDE_PARAM,
    Y_AMPLITUDE_PARAM,
    X_OFFSET_PARAM,
    Y_OFFSET_PARAM,
    // Center of Rotation X
    X_C_ROTATION_PARAM,
    // Center of Rotation Y
    Y_C_ROTATION_PARAM,
    ROTATION_PARAM,
    // Apply ABSOLUTE rotation or RELATIVE rotation (true/false)
    ROTATION_ABS_PARAM,
    // In the other screen, if the top bar shows or not
    TOP_BAR_UI_PARAM
};

#define FREQ_PARAM_SCALING  2
#define FREQ_SCALING        100

// #define TROWA_FREQ_KNOB_MIN     0
// #define TROWA_FREQ_KNOB_MAX     32000
// #define TROWA_FREQ_KNOB_DEF     261

#define TROWA_FREQ_KNOB_MIN     -5
#define TROWA_FREQ_KNOB_MAX      5
#define TROWA_FREQ_KNOB_DEF      0

#define TROWA_FREQ_VOLTAGE_MIN     -5
#define TROWA_FREQ_VOLTAGE_MAX      5
#define TROWA_FREQ_VOLTAGE_DEF      0

static char const * const enumStringsOnOff[] = {
	"Off",
	"On",
};

static const _NT_parameter	parameters[] = {
    //{ .name = "name", .min = MIN, .max = MAX, .def = DEF, .unit = UNIT, .scaling = 0, .enumStrings = NULL },
    NT_PARAMETER_AUDIO_INPUT( "Frequency Input", 1, 1 )
    NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output X", 1, 13 )	
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output Y", 1, 14 )
    { .name = "Frequency", 
        .min = TROWA_FREQ_KNOB_MIN * FREQ_SCALING, .max = TROWA_FREQ_KNOB_MAX * FREQ_SCALING, .def = TROWA_FREQ_KNOB_DEF * FREQ_SCALING, 
        .unit = kNT_unitVolts, .scaling = FREQ_PARAM_SCALING, .enumStrings = NULL },
    // { .name = "Frequency", 
    //     .min = TROWA_FREQ_KNOB_MIN, .max = TROWA_FREQ_KNOB_MAX, .def = TROWA_FREQ_KNOB_DEF, 
    //     .unit = kNT_unitHz, .scaling = 0, .enumStrings = NULL },
    { .name = "# Sides", 
        .min = TS_POLYGEN_VERTICES_MIN, .max = TS_POLYGEN_VERTICES_MAX, .def = TS_POLYGEN_VERTICES_DEF, 
        .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Angle Offset", 
        .min = TS_POLYGEN_ANGLE_OFFSET_DEG_MIN, .max = TS_POLYGEN_ANGLE_OFFSET_DEG_MAX, .def = TS_POLYGEN_ANGLE_OFFSET_DEG_DEF, 
        .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    // { .name = "Inner Radius Size", 
    //     .min = MIN_PARAMETER_VAL, 
    //     .max = MAX_PARAMETER_VAL, 
    //     .def = scale_i(TS_POLYGEN_INNER_RADIUS_MULT_DEF, TS_POLYGEN_INNER_RADIUS_MULT_MIN, TS_POLYGEN_INNER_RADIUS_MULT_MAX, MIN_PARAMETER_VAL, MAX_PARAMETER_VAL),
    //     .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Inner Radius Size", 
        .min = static_cast<int16_t>(TS_POLYGEN_INNER_RADIUS_MULT_MIN * 100), 
        .max = static_cast<int16_t>(TS_POLYGEN_INNER_RADIUS_MULT_MAX * 100), 
        .def = static_cast<int16_t>(TS_POLYGEN_INNER_RADIUS_MULT_DEF * 100),
        .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    // { .name = "Inner Radius Angle Offset", 
    //     .min = MIN_PARAMETER_VAL, //TS_POLYGEN_INNER_OFFSET_DEG_MIN, 
    //     .max = MAX_PARAMETER_VAL, //TS_POLYGEN_INNER_OFFSET_DEG_MAX, 
    //     .def = scale_i(TS_POLYGEN_INNER_OFFSET_DEG_DEF, TS_POLYGEN_INNER_OFFSET_DEG_MIN, TS_POLYGEN_INNER_OFFSET_DEG_MAX), 
    //     .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Inner Radius Angle Offset", 
        .min = static_cast<int16_t>(TS_POLYGEN_INNER_OFFSET_DEG_MIN * 100), 
        .max = static_cast<int16_t>(TS_POLYGEN_INNER_OFFSET_DEG_MAX * 100), 
        .def = static_cast<int16_t>(TS_POLYGEN_INNER_OFFSET_DEG_DEF * 100), 
        .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "X Radius/Amplitude", 
        .min = static_cast<int16_t>(TS_POLYGEN_AMPL_MIN * VOLTAGE_SCALING), 
        .max = static_cast<int16_t>(TS_POLYGEN_AMPL_MAX * VOLTAGE_SCALING), 
        .def = static_cast<int16_t>(TS_POLYGEN_AMPL_DEF * VOLTAGE_SCALING), 
        .unit = kNT_unitVolts, .scaling = VOLTAGE_PARAM_SCALING, .enumStrings = NULL },
    { .name = "Y Radius/Amplitude", 
        .min = static_cast<int16_t>(TS_POLYGEN_AMPL_MIN * VOLTAGE_SCALING), 
        .max = static_cast<int16_t>(TS_POLYGEN_AMPL_MAX * VOLTAGE_SCALING), 
        .def = static_cast<int16_t>(TS_POLYGEN_AMPL_DEF * VOLTAGE_SCALING), 
        .unit = kNT_unitVolts, .scaling = VOLTAGE_PARAM_SCALING, .enumStrings = NULL },
    { .name = "X Offset", 
        .min = static_cast<int16_t>(TS_POLYGEN_AMPL_MIN * VOLTAGE_SCALING), 
        .max = static_cast<int16_t>(TS_POLYGEN_AMPL_MAX * VOLTAGE_SCALING), 
        .def = 0, 
        .unit = kNT_unitVolts, .scaling = VOLTAGE_PARAM_SCALING, .enumStrings = NULL },
    { .name = "Y Offset", 
        .min = static_cast<int16_t>(TS_POLYGEN_AMPL_MIN * VOLTAGE_SCALING), 
        .max = static_cast<int16_t>(TS_POLYGEN_AMPL_MAX * VOLTAGE_SCALING), 
        .def = 0, 
        .unit = kNT_unitVolts, .scaling = VOLTAGE_PARAM_SCALING, .enumStrings = NULL },
    { .name = "X Center of Rotation", 
        .min = static_cast<int16_t>(TS_POLYGEN_AMPL_MIN * VOLTAGE_SCALING), 
        .max = static_cast<int16_t>(TS_POLYGEN_AMPL_MAX * VOLTAGE_SCALING), 
        .def = 0, 
        .unit = kNT_unitVolts, .scaling = VOLTAGE_PARAM_SCALING, .enumStrings = NULL },
    { .name = "Y Center of Rotation", 
        .min = static_cast<int16_t>(TS_POLYGEN_AMPL_MIN * VOLTAGE_SCALING), 
        .max = static_cast<int16_t>(TS_POLYGEN_AMPL_MAX * VOLTAGE_SCALING), 
        .def = 0, 
        .unit = kNT_unitVolts, .scaling = VOLTAGE_PARAM_SCALING, .enumStrings = NULL },
    { .name = "Rotation", 
        .min = static_cast<int16_t>( TS_POLGEN_ROT_DEG_MIN ), 
        .max = static_cast<int16_t>( TS_POLGEN_ROT_DEG_MAX ), 
        .def = static_cast<int16_t>( TS_POLGEN_ROT_DEG_DEF ), 
        .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Spin", .min = 0, .max = 1, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = enumStringsOnOff },
    { .name = "Top Bar", .min = 0, .max = 1, .def = 1, .unit = kNT_unitNone, .scaling = 0, .enumStrings = enumStringsOnOff }
};

//static const uint8_t routingParams[] = { kParamOutput, kParamOutputMode };

static const uint8_t page1[] = { FREQ_PARAM,
    // Number of outer vertices. 'Inner' vertices will be mapped in between, but by default will be in-line with the outer vertices.
    NUM_VERTICES_PARAM,
    // Angle offset for shape / Initial rotation
    ANGLE_OFFSET_PARAM,
    // Radius of inner vertices relative to the outer radius. Default is 1 (no star).
    INNER_VERTICES_RADIUS_PARAM,
    // Angle offset of the inner vertices. Default is 0 degrees from 180/N (mid).
    INNER_VERTICES_ANGLE_PARAM,
    X_AMPLITUDE_PARAM,
    Y_AMPLITUDE_PARAM,
    X_OFFSET_PARAM,
    Y_OFFSET_PARAM,
    ROTATION_PARAM,
    // Center of Rotation X
    X_C_ROTATION_PARAM,
    // Center of Rotation Y
    Y_C_ROTATION_PARAM,
    // Apply ABSOLUTE rotation or RELATIVE rotation (true/false)
    ROTATION_ABS_PARAM,
    TOP_BAR_UI_PARAM
};
// Page 2: Routing X output
static const uint8_t page2[] = { kParamInput, kParamOutput, kParamOutputMode, kParamOutput2, kParamOutputMode2 };

static const _NT_parameterPage pages[] = {
	{ .name = "Polygon", .numParams = ARRAY_SIZE(page1), .params = page1 },
	{ .name = "Routing", .numParams = ARRAY_SIZE(page2), .params = page2 }
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

void	calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_polyGenAlgorithm);
	req.dram = 0;
	req.dtc = 0;
	req.itc = 0;
}

_NT_algorithm*	construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req,const int32_t* specifications )
{
    //_polyGenAlgorithm* alg = new (ptrs.sram) _polyGenAlgorithm((_polyGenAlgorithm_DTC*)ptrs.dtc );
    _polyGenAlgorithm* alg = new (ptrs.sram) _polyGenAlgorithm();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	return alg;
}

// Gets the frequency from the voltage (1V per octave)
float getFrequencyFromVoltage(float voltage){
    // 1V per Octave
    return BASE_FREQ_HZ*powf(2.0f, voltage);
}

void	parameterChanged( _NT_algorithm* self, int p )
{
	_polyGenAlgorithm* pThis = (_polyGenAlgorithm*)self;
    //_polyGenInputs* dtc = pThis->dtc;
   
    switch (p)
    {
        case ParamIds::FREQ_PARAM:
            // Frequency parameter
            pThis->frequencyParam_V = static_cast<float>(pThis->v[FREQ_PARAM])/FREQ_SCALING;    
            // clamp
            pThis->frequencyParam_V = clamp(pThis->frequencyParam_V, static_cast<float>(TROWA_FREQ_KNOB_MIN), static_cast<float>(TROWA_FREQ_KNOB_MAX));
            break;
        case ParamIds::NUM_VERTICES_PARAM:
            pThis->numVertices = static_cast<uint8_t>( pThis->v[NUM_VERTICES_PARAM] );
            break;
        case ParamIds::ANGLE_OFFSET_PARAM:
            pThis->angleOffset_rad = pThis->v[ANGLE_OFFSET_PARAM] * PI / 180.0f;
            break;
        case ParamIds::ROTATION_ABS_PARAM:
            pThis->rotationIsAbs = !(pThis->v[ROTATION_ABS_PARAM] > 0);
            if (pThis->rotationIsAbs){
                // Re-read the rotation parameter
                pThis->rotation_deg = -1.0f * static_cast<float>(pThis->v[ROTATION_PARAM]);
                // Just make rotation simplier (-360 to 360)
                if (pThis->rotation_deg < -360 || pThis->rotation_deg > 360)
                {
                    int n = static_cast<int>( std::abs(pThis->rotation_deg) / 360.0f + 0.5f );
                    if (pThis->rotation_deg >= 0.0f)
                        pThis->rotation_deg -= (n * 360);
                    else
                        pThis->rotation_deg += (n * 360);
                }
                pThis->rotation_rad = pThis->rotation_deg / 180.0f * PI;   
            }
            break;
        case ParamIds::INNER_VERTICES_RADIUS_PARAM:
            pThis->innerRadiusMult = static_cast<float>(pThis->v[INNER_VERTICES_RADIUS_PARAM]) / 100.f; 
            {
                // See if we even have to worry about inner (2ndary) vertices (ignore if very close to 100%)
                const float threshold = 0.0005f;
                float radiusDiff = 1.0f - pThis->innerRadiusMult;
                pThis->useInnerVerts = radiusDiff < -threshold || radiusDiff > threshold;
            }
            break;
        case ParamIds::INNER_VERTICES_ANGLE_PARAM:
            pThis->innerAngleMult = static_cast<float>(pThis->v[INNER_VERTICES_ANGLE_PARAM]) / 100.f;
            break;
        case ParamIds::X_AMPLITUDE_PARAM:
        case ParamIds::Y_AMPLITUDE_PARAM:
        case ParamIds::X_OFFSET_PARAM:
        case ParamIds::Y_OFFSET_PARAM:
        case ParamIds::X_C_ROTATION_PARAM:
        case ParamIds::Y_C_ROTATION_PARAM:
            {
                //=== * Amplitude, Offset for X & Y * ===
                //const int numVoltageParams = 6;
                float* vPtrs[] = { &(pThis->xAmpl), &(pThis->yAmpl), &(pThis->xOffset), &(pThis->yOffset), &(pThis->xCRot), &(pThis->yCRot) };
                //int vParamIds[] = { X_AMPLITUDE_PARAM, Y_AMPLITUDE_PARAM, X_OFFSET_PARAM, Y_OFFSET_PARAM, X_C_ROTATION_PARAM, Y_C_ROTATION_PARAM };
                (*(vPtrs[p - X_AMPLITUDE_PARAM])) = clamp(static_cast<float>(pThis->v[p])/VOLTAGE_SCALING, TS_POLYGEN_AMPL_MIN, TS_POLYGEN_AMPL_MAX);
            }
            break;
        case ParamIds::ROTATION_PARAM:
            {
                //=== * Rotation * ===
                float rot_deg = -1.0f * static_cast<float>(pThis->v[ROTATION_PARAM]);    
                if (pThis->rotationIsAbs)
                {
                    pThis->rotation_deg = rot_deg;
                }
                else
                {
                    // Rotations is N deg/second
                    // So need to reduce by sample rate
                    uint32_t sRate = (NT_globals.sampleRate > 0) ? NT_globals.sampleRate : 1000;
                    pThis->rotation_deg += rot_deg / static_cast<float>(sRate);
                }
                // Just make rotation simplier (-360 to 360)
                if (pThis->rotation_deg < -360 || pThis->rotation_deg > 360)
                {
                    int n = static_cast<int>( std::abs(pThis->rotation_deg) / 360.0f + 0.5f );
                    if (pThis->rotation_deg >= 0.0f)
                        pThis->rotation_deg -= (n * 360);
                    else
                        pThis->rotation_deg += (n * 360);
                }
                pThis->rotation_rad = pThis->rotation_deg / 180.0f * PI;

            }
            break;
        case ParamIds::TOP_BAR_UI_PARAM:
            pThis->topBarOn = pThis->v[p] > 0;
            break;
        default:
            break;
    }
    return;
}

void 	step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_polyGenAlgorithm* pThis = (_polyGenAlgorithm*)self;
    //_polyGenInputs* dtc = pThis->dtc;
    int numFrames = numFramesBy4 * 4;


    
    //=== * Timing/Frequency *===
    float freq = pThis->frequencyParam_V;
    const float* in = busFrames + ( pThis->v[kParamInput] - 1 ) * numFrames;
    float* out1 = busFrames + ( pThis->v[kParamOutput] - 1 ) * numFrames;
    float* out2 = busFrames + ( pThis->v[kParamOutput2] - 1 ) * numFrames;

    for (int frame = 0; frame < numFrames; ++frame)
    {
        //=== * Rotation * ===
        if (!pThis->rotationIsAbs)
        {
            //----------------------------------------------------------
            // Current rotation needs to be calculated every time then.
            //----------------------------------------------------------
            // Rotations is N deg/second
            // So need to reduce by sample rate
            float rot_deg = -1.0f * static_cast<float>(pThis->v[ROTATION_PARAM]);    
            uint32_t sRate = (NT_globals.sampleRate > 0) ? NT_globals.sampleRate : 1000;
            pThis->rotation_deg += rot_deg / static_cast<float>(sRate);

            // Just make rotation simplier (-360 to 360)
            if (pThis->rotation_deg < -360 || pThis->rotation_deg > 360)
            {
                int n = static_cast<int>( std::abs(pThis->rotation_deg) / 360.0f + 0.5f );
                if (pThis->rotation_deg >= 0.0f)
                    pThis->rotation_deg -= (n * 360);
                else
                    pThis->rotation_deg += (n * 360);
            }
            pThis->rotation_rad = pThis->rotation_deg / 180.0f * PI;
        }
        //=== * Main Clock * ===
        // Main Clock:
        float vertexTime, nextVertexTime; // Normalized 0 to 1 time for vertices.

        float input = in[frame] + freq;
        input = clamp(input, static_cast<float>(TROWA_FREQ_KNOB_MIN), static_cast<float>(TROWA_FREQ_KNOB_MAX));
        float f = powf(2.0f, input) * BASE_FREQ_HZ * pThis->numVertices;
        
        // Want to draw N polygons per second (so multiply by # vertices):
        float clockTime = f;//   powf(2.0, input) * TROWA_FREQ_KNOB_MULT * pThis->numVertices;
        float dt = clockTime / NT_globals.sampleRate; // Real dt
        pThis->phase += dt; // Main vertex phase
        pThis->innerPhase += dt; // 2ndary/Inner vertex phase
        
        // Check for Next Side/Vertex
        bool newCorner = false;
        bool syncIn = false;

        if (pThis->innerPhase >= 1.0f)
        {
            pThis->innerPhase = 0.0f;
        }
        
        if (pThis->phase >= 1.0f || syncIn)
        {
            if (syncIn)
            {
                pThis->phase = 0.0f;
                pThis->currVertexIx = 0;
            }
            else
            {
                pThis->phase -= 1.0f; // (Soft) Reset main clock phase
                pThis->currVertexIx++;
            }
            newCorner = true;
            
            if (pThis->currVertexIx >= pThis->numVertices)
                pThis->currVertexIx = 0;

            pThis->innerPhase = 0; // (Hard) Reset inner/2ndary phase (for inner/2ndary vertices)
            pThis->innerSideIx = 0; // Reset the side we are on (for inner/2ndary vertices)
        }

        // Which vertex we are on (outer/main)
        if (pThis->currVertexIx >= pThis->numVertices)
            pThis->currVertexIx = 0;
        pThis->nextVertexIx = pThis->currVertexIx + 1;
        if (pThis->nextVertexIx >= pThis->numVertices)
            pThis->nextVertexIx = 0;

        //=======================================
        // Calculate the 2 vertices we will use
        //=======================================
        
        // Time/Phase for sin
        vertexTime = static_cast<float>(pThis->currVertexIx) / static_cast<float>(pThis->numVertices);
        nextVertexTime = static_cast<float>(pThis->nextVertexIx) / static_cast<float>(pThis->numVertices);
        
        float v1Time = vertexTime;
        float v2Time = nextVertexTime;
        Vec v1Ampl = Vec(pThis->xAmpl, pThis->yAmpl);
        Vec v2Ampl = Vec(pThis->xAmpl, pThis->yAmpl);
        float linearPhase = clamp(pThis->phase, 0.0f, 1.0f); // For interpolation
        Vec thisCorner, nextCorner;
        
        //---------------------
        // This Corner/Vertex
        //---------------------
        thisCorner.x = v1Ampl.x * SINFUNC( 2 * PI * v1Time + pThis->angleOffset_rad);
        thisCorner.y = v1Ampl.y * COSFUNC( 2 * PI * v1Time + pThis->angleOffset_rad);

        //------------------------
        // The Next Corner/Vertex
        //------------------------
        nextCorner.x = v2Ampl.x * SINFUNC( 2 * PI * v2Time + pThis->angleOffset_rad);
        nextCorner.y = v2Ampl.y * COSFUNC( 2 * PI * v2Time + pThis->angleOffset_rad);
        
        if (pThis->useInnerVerts)
        {
            // Inject inner/2ndary vertex
            float iTime = 0.5f * (1 + pThis->innerAngleMult);
                    
            // Use our inner/2ndary phase to see where we are
            linearPhase = clamp(pThis->innerPhase, 0.0f, 1.0f);
            
#if TS_POLYGEN_IRADIUS_REL_2_MID_POINT
            // Calculate the point on the line between the two corners
            float midX = thisCorner.x + (nextCorner.x - thisCorner.x) * 0.5f;
            float midY = thisCorner.y + (nextCorner.y - thisCorner.y) * 0.5f;
            float ampl = SQRTFUNC(midX * midX + midY * midY) * pThis->innerRadiusMult;
#endif
            if (linearPhase < 0.5f)
            {
                // First Vertex then this middle inner one
                v2Time = vertexTime + iTime/pThis->numVertices;
#if TS_POLYGEN_IRADIUS_REL_2_MID_POINT
                v2Ampl.x = ampl * SGN(v2Ampl.x);
                v2Ampl.y = ampl * SGN(v2Ampl.y);
#else
                v2Ampl.x *= pThis->innerRadiusMult;
                v2Ampl.y *= pThis->innerRadiusMult;
#endif
                linearPhase = linearPhase / iTime; // Rescale 0 to 1
                nextCorner.x = v2Ampl.x * SINFUNC( 2 * PI * v2Time + pThis->angleOffset_rad);
                nextCorner.y = v2Ampl.y * COSFUNC( 2 * PI * v2Time + pThis->angleOffset_rad);
            }
            else
            {
                // This middle inner one and then the 2nd vertex
                v1Time = vertexTime + iTime / pThis->numVertices;
#if TS_POLYGEN_IRADIUS_REL_2_MID_POINT
                v1Ampl.x = ampl * SGN(v1Ampl.x);
                v1Ampl.y = ampl * SGN(v1Ampl.y);
#else
                v1Ampl.x *= pThis->innerRadiusMult;
                v1Ampl.y *= pThis->innerRadiusMult;
#endif
                linearPhase = (linearPhase - 0.5f) / 0.5f;    // Rescale 0 to 1
                thisCorner.x = v1Ampl.x * SINFUNC( 2 * PI * v1Time + pThis->angleOffset_rad);
                thisCorner.y = v1Ampl.y * COSFUNC( 2 * PI * v1Time + pThis->angleOffset_rad);
            }
        } // end if inner/2ndary vertices
        
        //===============================
        // Interpolate this step's value
        //===============================
        // Interpolate based on which point we are on this side
        float vx = thisCorner.x;
        float vy = thisCorner.y;
        if (!newCorner)
        {
            // We don't have to interpolate if it is a new corner, otherwise simple linear interpolation
            float mult = clamp(linearPhase, 0.0f, 1.0f);
            vx += (nextCorner.x - thisCorner.x) * mult;
            vy += (nextCorner.y - thisCorner.y) * mult;
        }
        
        //===============================
        // Rotate the point
        //===============================
        float vxR, vyR;
        vxR = vx;
        vyR = vy;
        
        if (pThis->rotation_deg != 0 && pThis->rotation_deg != 360)
        {
            float sinrot = SINFUNC( pThis->rotation_rad );
            float cosrot = COSFUNC( pThis->rotation_rad );
            
            // Translate to rotation center
            vx -= pThis->xCRot;
            vy -= pThis->yCRot;
            
            // Rotate
            vxR = vx * cosrot - vy * sinrot;
            vyR = vx * sinrot + vy * cosrot;
            
            // Translate back after rotation
            vxR += pThis->xCRot;
            vyR += pThis->yCRot;
        }
        
        //================================
        // Post Rotation Offset
        //================================
        vxR += pThis->xOffset;
        vyR += pThis->yOffset;

        //================================
        // Outputs
        //================================
        
        // CHANNEL 0 (X)
        uint8_t ch = 0;
        out1[frame] = vxR;
        // CHANNEL 1 (Y)
        ch = 1;
        out2[frame] = vyR;
    }

    // for ( uint32_t ch=0; ch < 2; ++ch )
	// {
    //     for ( int i = 0; i < numFrames; ++i )
    //     {
    //         out[i] = (ch == 1) ? vyR : vxR ;
    //     }
	// 	out += numFrames;
	// }
    // float* out = busFrames + ( pThis->v[kParamOutput] - 1 ) * numFrames;
    // for ( uint32_t ch=0; ch < 2; ++ch )
	// {
    //     for ( int i = 0; i < numFrames; ++i )
    //     {
    //         out[i] = (ch == 1) ? vyR : vxR ;
    //     }
	// 	out += numFrames;
	// }

    return;    
}

void drawShape(Vec* buffer, int bufferLen, Vec offset, float mult, float lWidth, int lColor)
{
    //NT_drawShapeF( _NT_shape shape, float x0, float y0, float x1, float y1, float colour=15 );
    float x, y, lastX, lastY;
	for (int v = 0; v < bufferLen; v++)
	{
        x = mult*buffer[v].x + offset.x;
        y = mult*buffer[v].y + offset.y;
		if (v > 0)
        {
            NT_drawShapeF(kNT_line, lastX, lastY, x, y, lColor);
        }
        else 
        {
            // Go from the last point to the first point
            lastX = mult*buffer[bufferLen - 1].x + offset.x;
            lastY = mult*buffer[bufferLen - 1].y + offset.y;
            NT_drawShapeF(kNT_line, lastX, lastY, x, y, lColor);
        }
        lastX = x;
        lastY = y;
	} // end loop through vertices
    // x = mult*buffer[0].x + offset.x;
    // y = mult*buffer[0].y + offset.y;
    // NT_drawShapeF(kNT_line, lastX, lastY, x, y, lColor);
	return;
}

#define SHOW_RAW_BUFFER     0

bool	draw( _NT_algorithm* self )
{
	_polyGenAlgorithm* pThis = (_polyGenAlgorithm*)self;
	
	// for ( int i=0; i<pThis->v[kParamGain]; ++i )
	// 	NT_screen[ 128 * 20 + i ] = 0xa5;
		
    //NT_drawText( int x, int y, const char* str, int colour=15, _NT_textAlignment align=kNT_textLeft, _NT_textSize size=kNT_textNormal );
	NT_drawText( 10, 40, "polyGen" );
	//NT_drawText( 256, 64, "ho0oOrt!", 8, kNT_textRight, kNT_textLarge );
	// draw a shape (float coordinates, antialiased)
    //void		NT_drawShapeF( _NT_shape shape, float x0, float y0, float x1, float y1, float colour=15 );
	//NT_drawShapeI( kNT_line, 20, 45, 50, 55, 8 );
	//NT_drawShapeF( kNT_line, 20, 50, 50, 60 );

    // screen is 256x64 - each byte contains two pixels
    Vec boxSize = Vec(256, 64);
    float padding = 2.0f;
    int lineWidth = 1;
    int lineColor = 17;


    int numVertices = pThis->numVertices;// TS_POLYGEN_VERTICES_DEF;
	float innerRadiusMult = pThis->innerRadiusMult;// TS_POLYGEN_INNER_RADIUS_MULT_DEF;
	float innerAngleMult = pThis->innerAngleMult;// TS_POLYGEN_INNER_OFFSET_DEG_DEF;
	float xAmpl = pThis->xAmpl;// TS_POLYGEN_AMPL_DEF;
	float yAmpl = pThis->yAmpl;// TS_POLYGEN_AMPL_DEF;
	bool useInnerVerts = pThis->useInnerVerts;
	float rotation_rad = pThis->rotation_rad;// 0.0f;
	float angleOffset_rad = pThis->angleOffset_rad;// 0.0f;
	float in_range[2] = { TS_POLYGEN_AMPL_MIN, TS_POLYGEN_AMPL_MAX };
	Vec rotCenter = Vec(pThis->xCRot, pThis->yCRot);
	float xOff = pThis->xOffset;
	float yOff = pThis->yOffset;

    // Calculate canvas box dimension
	float canvasCenterX = boxSize.x / 2.0f; // Center (0, 0)
	float canvasCenterY = boxSize.y / 2.0f; // Center (0, 0)
	float dim = 0.0f;
	// Center and make sure to scale
	if (boxSize.y > boxSize.x)
	{
		dim = boxSize.x;
	}
	else
	{
		dim = boxSize.y;
	}
	dim -= padding * 2;
	// Rescale amplitudes to fit in box
	float canvasRadius = dim / 2.0f;
	xAmpl = scale(xAmpl, in_range[0], in_range[1], -canvasRadius, canvasRadius);
	yAmpl = scale(-yAmpl, in_range[0], in_range[1], -canvasRadius, canvasRadius); // invert Y
	rotCenter.x = scale(rotCenter.x, in_range[0], in_range[1], -canvasRadius, canvasRadius);
	rotCenter.y = scale(-rotCenter.y, in_range[0], in_range[1], -canvasRadius, canvasRadius);	 // invert Y
	xOff = scale(xOff, in_range[0], in_range[1], -canvasRadius, canvasRadius);
	yOff = scale(-yOff, in_range[0], in_range[1], -canvasRadius, canvasRadius); // invert Y
	Vec innerAmpl = Vec(xAmpl * innerRadiusMult, yAmpl * innerRadiusMult);

	//============================
	// Calculate the Buffers
	//============================
	Vec nextPoint;
	bool nextPointCalc = false;
	float sinrot = SINFUNC(rotation_rad);
	float cosrot = COSFUNC(rotation_rad);
	float n = static_cast<float>(numVertices);
	float innerAngleShift = 0.5f * (1 + innerAngleMult) / n;
	int ix = 0;
	Vec maxVals = Vec(0, 0);

	int numPoints = (useInnerVerts) ? numVertices * 2 : numVertices;
#if SHOW_RAW_BUFFER
    Vec rawBuffer[BUFF_SIZE];
#else
    Vec rawPoint0;
#endif
    Vec rotatedBuffer[BUFF_SIZE];

	for (int v = 0; v < numVertices; v++)
	{
		Vec point;
		float pAngle = static_cast<float>(v) / n;
		if (nextPointCalc)
		{
			point = nextPoint;
		}
		else
		{
			point.x = xAmpl * SINFUNC(pAngle * 2 * PI + angleOffset_rad);
			point.y = yAmpl * COSFUNC(pAngle * 2 * PI + angleOffset_rad);
		}
		// Non-Rotated
#if SHOW_RAW_BUFFER        
		rawBuffer[ix].x = point.x;
		rawBuffer[ix].y = point.y;
#else
        if (ix == 0)
        {
            rawPoint0 = point;
        }
#endif
		if (point.x < 0 && -point.x > maxVals.x)
			maxVals.x = -point.x;
		else if (point.x > maxVals.x)
			maxVals.x = point.x;
		if (point.y < 0 && -point.y > maxVals.y)
			maxVals.y = -point.y;
		else if (point.y > maxVals.y)
			maxVals.y = point.y;

		// Rotate
		rotatedBuffer[ix].x = (point.x - rotCenter.x) * cosrot + (point.y - rotCenter.y) * sinrot + rotCenter.x;
		rotatedBuffer[ix].y = (-point.x - rotCenter.y) * sinrot + (point.y - rotCenter.y) * cosrot + rotCenter.y;
		ix++;

		if (useInnerVerts)
		{
			//------------------------
			// Calculate 2ndary point
			//------------------------
#if TS_POLYGEN_IRADIUS_REL_2_MID_POINT
			// Find amplitude/radius of mid point between the two corners
			int v2 = v + 1;
			if (v2 >= numVertices)
			{
#if SHOW_RAW_BUFFER                
				nextPoint = rawBuffer[0];
#else
                nextPoint = rawPoint0;
#endif
			}
			else
			{
				float p2Angle = static_cast<float>(v2) / n;
				nextPoint.x = xAmpl * SINFUNC(p2Angle * 2 * PI + angleOffset_rad);
				nextPoint.y = yAmpl * COSFUNC(p2Angle * 2 * PI + angleOffset_rad);
				nextPointCalc = true;
			}
			float midX = point.x + (nextPoint.x - point.x) * 0.5f;
			float midY = point.y + (nextPoint.y - point.y) * 0.5f;
			float ampl = SQRTFUNC(midX * midX + midY * midY) * innerRadiusMult;
			innerAmpl.x = ampl * SGN(xAmpl);
			innerAmpl.y = ampl * SGN(yAmpl);
#endif
			pAngle += innerAngleShift;
			point.x = innerAmpl.x * SINFUNC(pAngle * 2 * PI + angleOffset_rad);
			point.y = innerAmpl.y * COSFUNC(pAngle * 2 * PI + angleOffset_rad);
			if (point.x < 0 && -point.x > maxVals.x)
				maxVals.x = -point.x;
			else if (point.x > maxVals.x)
				maxVals.x = point.x;
			if (point.y < 0 && -point.y > maxVals.y)
				maxVals.y = -point.y;
			else if (point.y > maxVals.y)
				maxVals.y = point.y;

#if SHOW_RAW_BUFFER
			// Non-Rotated
			rawBuffer[ix].x = point.x;
			rawBuffer[ix].y = point.y;
#endif

			// Rotate
			rotatedBuffer[ix].x = (point.x - rotCenter.x) * cosrot + (point.y - rotCenter.y) * sinrot + rotCenter.x;
			rotatedBuffer[ix].y = (-point.x - rotCenter.y) * sinrot + (point.y - rotCenter.y) * cosrot + rotCenter.y;
			ix++;
		} // end if useInnerVerts
	} // end loop through vertices


	//=============================================================
	// Draw main preview (Rotated and Translated)
	//=============================================================
	//float pad = padding - lineWidth;    
    Vec offset = Vec(canvasCenterX + xOff, canvasCenterY + yOff);
    drawShape(rotatedBuffer, numPoints, offset, /*mult*/ 1.0, lineWidth, lineColor);

    //=============================================================
	// Draw secondary small preview (pre-Rotation and translation)
	//=============================================================
#if SHOW_RAW_BUFFER
	// float maxVal = (maxVals.x > maxVals.y) ? maxVals.x : maxVals.y;
	// float smMult = 0.3f * canvasRadius / maxVal;
	// offset.x = 0.85f * boxSize.x - padding; // Right
	// offset.y = 0.85f * boxSize.y - padding; // Bottom	
	// drawShape(args, rawBuffer, numPoints, offset, /*mult*/ smMult, lineWidth, secondColor);
#endif

	return pThis->topBarOn;
}

static const _NT_factory factory = 
{
	.guid = NT_MULTICHAR( 't', 'S', 'p', 'G' ),
	.name = "polyGen",
	.description = "Generates a polygon",
    .numSpecifications = 0,
    .specifications = NULL,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = parameterChanged,
	.step = step,
	.draw = draw,
};

uintptr_t pluginEntry( _NT_selector selector, uint32_t data )
{
	switch ( selector )
	{
	case kNT_selector_version:
		return kNT_apiVersionCurrent;
	case kNT_selector_numFactories:
		return 1;
	case kNT_selector_factoryInfo:
		return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
