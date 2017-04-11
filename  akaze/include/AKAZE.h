
/**
 * @file AKAZE.h
 * @brief Main class for detecting and computing binary descriptors in an
 * accelerated nonlinear scale space
 * @date Mar 27, 2013
 * @author Pablo F. Alcantarilla, Jesus Nuevo
 */

#ifndef _AKAZE_H_
#define _AKAZE_H_

//*************************************************************************************
//*************************************************************************************

// Includes
#include "config.h"
#include "fed.h"
#include "utils.h"
#include "nldiffusion_functions.h"

//*************************************************************************************
//*************************************************************************************

// AKAZE Class Declaration
class AKAZE
{

private:

    // Parameters of the AKAZE class
    int omin;               // Initial octave level
    int omax;               // Maximum octave level
    int noctaves;           // Number of octaves
    int nsublevels;         // Number of sublevels per octave level
    int img_width;          // Width of the original image
    int img_height;         // Height of the original image
    float soffset;          // Base scale offset
    float factor_size;      // Factor for the multiscale derivatives
    float sderivatives;     // Standard deviation of the Gaussian for the nonlinear diff. derivatives
    float kcontrast;        // The contrast parameter for the scalar nonlinear diffusion
    float dthreshold;       // Feature detector threshold response
    int diffusivity;        // Diffusivity type, 0->PM G1, 1->PM G2, 2-> Weickert
    int descriptor;     // Descriptor mode
    bool use_upright;	// Set to true in case we want to use the upright version of the descriptors
    int descriptor_size;    // Size of the descriptor in bits. Use 0 for the full descriptor
    int descriptor_pattern_size;    // Size of the pattern. Actual size sampled is 2*pattern_size
    int descriptor_channels;        // Number of channels to consider in the M-LDB descriptor
    bool save_scale_space; // For saving scale space images
    bool verbosity;	// Verbosity level
    std::vector<tevolution> evolution;	// Vector of nonlinear diffusion evolution

    // FED parameters
    int ncycles;                  // Number of cycles
    bool reordering;              // Flag for reordering time steps
    std::vector<std::vector<float > > tsteps;  // Vector of FED dynamic time steps
    std::vector<int> nsteps;      // Vector of number of steps per cycle

    // Some matrices for the M-LDB descriptor computation
    cv::Mat descriptorSamples;  // List of positions in the grids to sample LDB bits from.
    cv::Mat descriptorBits;
    cv::Mat bitMask;

    // Computation times variables in ms
    double tkcontrast;      // Kcontrast factor computation
    double tscale;          // Nonlinear Scale space generation
    double tderivatives;    // Multiscale derivatives
    double tdetector;       // Feature detector
    double textrema;        // Scale Space extrema
    double tsubpixel;       // Subpixel refinement
    double tdescriptor;     // Feature descriptors

public:

    // Constructor
    AKAZE(AKAZEOptions &options);

    // Destructor
    ~AKAZE(void);

    // Setters
    void Set_Octave_Min(int oaux){omin = oaux;}
    void Set_Octave_Max(int oaux){omax = oaux;}
    void Set_NSublevels(int naux){nsublevels = naux;}
    void Set_Save_Scale_Space_Flag(bool saux){save_scale_space = saux;}
    void Set_Image_Width(int waux){img_width = waux;}
    void Set_Image_Height(int haux){img_height = haux;}

    // Getters
    int Get_Image_Width(void){return img_width;}
    int Get_Image_Height(void){return img_height;}
    double Get_Time_KContrast(void){return tkcontrast;}
    double Get_Time_Scale_Space(void){return tscale;}
    double Get_Time_Derivatives(void){return tderivatives;}
    double Get_Time_Detector(void){return tdetector;}
    double Get_Time_Descriptor(void){return tdescriptor;}

    // Scale Space methods
    void Allocate_Memory_Evolution(void);
    int Create_Nonlinear_Scale_Space(const cv::Mat &img);
    void Feature_Detection(std::vector<cv::KeyPoint> &kpts);
    void Compute_Det_Hessian_Response(void);
    void Compute_Multiscale_Derivatives(void);
    void Determinant_Hessian_Parallel(std::vector<cv::KeyPoint> &kpts);
    void Find_Scale_Space_Extrema(std::vector<cv::KeyPoint> &kpts);
    void Do_Subpixel_Refinement(std::vector<cv::KeyPoint> &kpts);

    // Feature description methods
    void Compute_Descriptors(std::vector<cv::KeyPoint> &kpts, cv::Mat &desc);
    void Compute_Main_Orientation_SURF(cv::KeyPoint &kpt);

    // SURF Descriptor
    void Get_SURF_Descriptor_Upright_64(const cv::KeyPoint &kpt, float *desc);
    void Get_SURF_Descriptor_64(const cv::KeyPoint &kpt, float *desc);

    // M-SURF Descriptor
    void Get_MSURF_Upright_Descriptor_64(const cv::KeyPoint &kpt, float *desc);
    void Get_MSURF_Descriptor_64(const cv::KeyPoint &kpt, float *desc);

    // M-LDB Descriptor
    void Get_Upright_MLDB_Full_Descriptor(const cv::KeyPoint &kpt, unsigned char *desc);
    void Get_MLDB_Full_Descriptor(const cv::KeyPoint &kpt, unsigned char *desc);
    void Get_Upright_MLDB_Descriptor_Subset(const cv::KeyPoint &kpt, unsigned char *desc);
    void Get_MLDB_Descriptor_Subset(const cv::KeyPoint &kpt, unsigned char *desc);

    // Methods for saving some results and showing computation times
    void Save_Scale_Space(void);
    void Save_Detector_Responses(void);
    void Show_Computation_Times(void);
};

//*************************************************************************************
//*************************************************************************************

// Inline functions
void generateDescriptorSubsample(cv::Mat& sampleList, cv::Mat& comparisons,
                                 size_t nbits, size_t pattern_size, size_t nchannels);
float Get_Angle(float X, float Y);
float gaussian(float x, float y, float sig);
void Check_Descriptor_Limits(int &x, int &y, int width, int height );
int fRound(float flt);

//*************************************************************************************
//*************************************************************************************

#endif
