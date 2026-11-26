#include "gray_code_calibration.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace {
	constexpr int phaseSteps=4;
	constexpr double twoPi=2.0*CV_PI;

	double patch_mean(const cv::Mat& image,cv::Point2f point,int radius)
	{
		const int x=cvRound(point.x),y=cvRound(point.y);
		const cv::Rect bounds(0,0,image.cols,image.rows);
		const cv::Rect patch(x-radius,y-radius,2*radius+1,2*radius+1);
		const cv::Rect clipped=patch & bounds;
		if (clipped.width!=patch.width || clipped.height!=patch.height) return -1;
		return cv::mean(image(clipped))[0];
	}

	int gray_to_binary(int gray)
	{
		int binary=0;
		for (;gray;gray>>=1) binary^=gray;
		return binary;
	}
}

int Tinker::gray_code_bit_count(int extent)
{
	if (extent<=1) return 0;
	int bits=0;
	for (int value=extent-1;value>0;value>>=1) ++bits;
	return bits;
}

std::vector<cv::Mat> Tinker::make_gray_code_patterns(cv::Size projectorSize,int blockSize,int phasePeriod)
{
	if (phasePeriod<=0) phasePeriod=4*blockSize;
	if (projectorSize.width<=1 || projectorSize.height<=1 || blockSize<1 ||
		phasePeriod<blockSize || phasePeriod%blockSize!=0) return {};
	std::vector<cv::Mat> patterns;
	patterns.emplace_back(projectorSize,CV_8UC1,cv::Scalar(0));
	patterns.emplace_back(projectorSize,CV_8UC1,cv::Scalar(255));
	const int xCells=(projectorSize.width+blockSize-1)/blockSize;
	const int yCells=(projectorSize.height+blockSize-1)/blockSize;
	const int xBits=gray_code_bit_count(xCells);
	const int yBits=gray_code_bit_count(yCells);
	auto appendAxis=[&](bool horizontal,int bits) {
		for (int bit=bits-1;bit>=0;--bit) {
			cv::Mat normal(projectorSize,CV_8UC1);
			for (int y=0;y<projectorSize.height;++y) {
				auto* row=normal.ptr<unsigned char>(y);
				for (int x=0;x<projectorSize.width;++x) {
					const int coordinate=(horizontal ? x : y)/blockSize;
					const int gray=coordinate^(coordinate>>1);
					row[x]=((gray>>bit)&1) ? 255 : 0;
				}
			}
			patterns.push_back(normal);
			cv::Mat inverse;
			cv::bitwise_not(normal,inverse);
			patterns.push_back(inverse);
		}
	};
	appendAxis(true,xBits);
	appendAxis(false,yBits);
	// Gray code identifies the robust coarse cell. Four temporal sinusoidal
	// phase shifts per axis recover the position within that cell without
	// returning to fragile one-projector-pixel binary stripes.
	auto appendPhaseAxis=[&](bool horizontal) {
		for (int step=0;step<phaseSteps;++step) {
			cv::Mat pattern(projectorSize,CV_8UC1);
			const double phase=step*CV_PI/2.0;
			for (int y=0;y<projectorSize.height;++y) {
				auto* row=pattern.ptr<unsigned char>(y);
				for (int x=0;x<projectorSize.width;++x) {
					const int coordinate=horizontal ? x : y;
					const double local=(coordinate%phasePeriod+0.5)/phasePeriod;
					row[x]=cv::saturate_cast<unsigned char>(127.5+127.5*std::cos(twoPi*local+phase));
				}
			}
			patterns.push_back(pattern);
		}
	};
	appendPhaseAxis(true);
	appendPhaseAxis(false);
	return patterns;
}

bool Tinker::decode_gray_code_samples(const std::vector<cv::Mat>& frames,cv::Size projectorSize,int blockSize,int phasePeriod,
	const std::vector<cv::Point2f>& cameraSamples,const std::vector<cv::Point3f>& boardSamples,
	double minimumDirectContrast,double minimumBitContrast,int sampleRadius,
	std::vector<cv::Point2f>& projectorPoints,std::vector<cv::Point2f>& acceptedCamera,
	std::vector<cv::Point3f>& acceptedBoard,cv::Mat* debugImage,std::string& reason,
	GrayCodeDecodeDiagnostics* diagnostics)
{
	projectorPoints.clear(); acceptedCamera.clear(); acceptedBoard.clear();
	GrayCodeDecodeDiagnostics localDiagnostics;
	localDiagnostics.total=cameraSamples.size();
	if (phasePeriod<=0) phasePeriod=4*blockSize;
	if (blockSize<1 || phasePeriod<blockSize || phasePeriod%blockSize!=0) {
		reason="Gray/phase projector periods must be positive and phase period must be a multiple of block size"; return false;
	}
	const int xCells=(projectorSize.width+blockSize-1)/blockSize;
	const int yCells=(projectorSize.height+blockSize-1)/blockSize;
	const int xBits=gray_code_bit_count(xCells),yBits=gray_code_bit_count(yCells);
	const size_t grayFrameCount=2+2*static_cast<size_t>(xBits+yBits);
	const size_t expected=grayFrameCount+2*phaseSteps;
	if (frames.size()!=expected || cameraSamples.size()!=boardSamples.size() || frames.empty()) {
		reason="Gray-code frame/sample count mismatch"; return false;
	}
	for (const auto& frame:frames) if (frame.empty() || frame.type()!=CV_8UC1 || frame.size()!=frames.front().size()) {
		reason="Gray-code captures must be equal-sized grayscale images"; return false;
	}
	if (debugImage) cv::cvtColor(frames[1],*debugImage,cv::COLOR_GRAY2BGR);
	size_t patchFailures=0,directFailures=0,bitFailures=0,phaseFailures=0,consistencyFailures=0,rangeFailures=0;
	double minimumDirectObserved=255,maximumDirectObserved=0;
	double minimumBitObserved=255,maximumBitObserved=0;
	double minimumPhaseAmplitudeObserved=255,maximumPhaseAmplitudeObserved=0;
	for (size_t sample=0;sample<cameraSamples.size();++sample) {
		const double black=patch_mean(frames[0],cameraSamples[sample],sampleRadius);
		const double white=patch_mean(frames[1],cameraSamples[sample],sampleRadius);
		const bool patchValid=black>=0 && white>=0;
		const double direct=patchValid ? std::abs(white-black) : 0;
		minimumDirectObserved=std::min(minimumDirectObserved,direct);
		maximumDirectObserved=std::max(maximumDirectObserved,direct);
		const bool directValid=patchValid && direct>=minimumDirectContrast;
		bool bitValid=true;
		bool valid=directValid;
		int grayX=0,grayY=0;
		size_t frameIndex=2;
		auto decodeAxis=[&](int bits,int& gray) {
			for (int bit=bits-1;bit>=0;++frameIndex,++frameIndex,--bit) {
				const double normal=patch_mean(frames[frameIndex],cameraSamples[sample],sampleRadius);
				const double inverse=patch_mean(frames[frameIndex+1],cameraSamples[sample],sampleRadius);
				const double contrast=(normal>=0 && inverse>=0) ? std::abs(normal-inverse) : 0;
				minimumBitObserved=std::min(minimumBitObserved,contrast);
				maximumBitObserved=std::max(maximumBitObserved,contrast);
				if (normal<0 || inverse<0 || contrast<minimumBitContrast) { valid=false; bitValid=false; }
				if (normal>inverse) gray|=1<<bit;
			}
		};
		decodeAxis(xBits,grayX); decodeAxis(yBits,grayY);
		const int xCell=gray_to_binary(grayX),yCell=gray_to_binary(grayY);
		const bool rangeValid=xCell>=0 && xCell<xCells && yCell>=0 && yCell<yCells;
		auto decodePhase=[&](size_t firstFrame,double& fraction) {
			const double i0=patch_mean(frames[firstFrame],cameraSamples[sample],sampleRadius);
			const double i1=patch_mean(frames[firstFrame+1],cameraSamples[sample],sampleRadius);
			const double i2=patch_mean(frames[firstFrame+2],cameraSamples[sample],sampleRadius);
			const double i3=patch_mean(frames[firstFrame+3],cameraSamples[sample],sampleRadius);
			if (i0<0 || i1<0 || i2<0 || i3<0) return false;
			const double cosine=i0-i2;
			const double sine=i3-i1;
			const double amplitude=0.5*std::hypot(cosine,sine);
			minimumPhaseAmplitudeObserved=std::min(minimumPhaseAmplitudeObserved,amplitude);
			maximumPhaseAmplitudeObserved=std::max(maximumPhaseAmplitudeObserved,amplitude);
			if (amplitude<minimumBitContrast) return false;
			double angle=std::atan2(sine,cosine);
			if (angle<0) angle+=twoPi;
			fraction=angle/twoPi;
			return std::isfinite(fraction);
		};
		double phaseX=0,phaseY=0;
		const bool phaseXValid=decodePhase(grayFrameCount,phaseX);
		const bool phaseYValid=decodePhase(grayFrameCount+phaseSteps,phaseY);
		const bool phaseValid=phaseXValid && phaseYValid;
		const bool coarseValid=directValid && bitValid && rangeValid;
		auto unwrapPhase=[&](int cell,double fraction,int extent) {
			const double cellCenter=(cell+0.5)*blockSize-0.5;
			const double wrapped=fraction*phasePeriod-0.5;
			const double cycles=std::round((cellCenter-wrapped)/phasePeriod);
			return std::clamp(wrapped+cycles*phasePeriod,0.0,static_cast<double>(extent-1));
		};
		double refinedX=0,refinedY=0;
		bool consistent=false;
		if (phaseValid && rangeValid) {
			refinedX=unwrapPhase(xCell,phaseX,projectorSize.width);
			refinedY=unwrapPhase(yCell,phaseY,projectorSize.height);
			constexpr double tolerance=1.5;
			const double minX=xCell*blockSize-tolerance;
			const double maxX=std::min(projectorSize.width-1,(xCell+1)*blockSize-1)+tolerance;
			const double minY=yCell*blockSize-tolerance;
			const double maxY=std::min(projectorSize.height-1,(yCell+1)*blockSize-1)+tolerance;
			consistent=refinedX>=minX && refinedX<=maxX && refinedY>=minY && refinedY<=maxY;
		}
		valid=coarseValid && phaseValid && consistent;
		if (!patchValid) ++patchFailures;
		else if (!directValid) ++directFailures;
		if (!bitValid) ++bitFailures;
		if (!phaseValid) ++phaseFailures;
		else if (!consistent) ++consistencyFailures;
		if (!rangeValid) ++rangeFailures;
		if (coarseValid) ++localDiagnostics.coarseGrayValid;
		if (phaseValid) ++localDiagnostics.phaseValid;
		if (coarseValid && phaseValid && consistent) ++localDiagnostics.grayPhaseConsistent;
		if (valid) {
			const float x=static_cast<float>(refinedX);
			const float y=static_cast<float>(refinedY);
			projectorPoints.emplace_back(x,y);
			acceptedCamera.push_back(cameraSamples[sample]);
			acceptedBoard.push_back(boardSamples[sample]);
		}
		if (debugImage) cv::circle(*debugImage,cameraSamples[sample],3,valid ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255),-1,cv::LINE_AA);
	}
	std::ostringstream summary;
	summary << "decoded " << projectorPoints.size() << '/' << cameraSamples.size()
		<< " Gray-code points; patch_oob=" << patchFailures
		<< ", reference_contrast_fail=" << directFailures
		<< ", bit_contrast_fail=" << bitFailures
		<< ", phase_contrast_fail=" << phaseFailures
		<< ", gray_phase_consistency_fail=" << consistencyFailures
		<< ", decoded_range_fail=" << rangeFailures
		<< ", coarse_gray_valid=" << localDiagnostics.coarseGrayValid << '/' << localDiagnostics.total
		<< ", phase_valid=" << localDiagnostics.phaseValid << '/' << localDiagnostics.total
		<< ", gray_phase_consistent=" << localDiagnostics.grayPhaseConsistent << '/' << localDiagnostics.total
		<< ", observed_reference_contrast=[" << minimumDirectObserved << ',' << maximumDirectObserved << ']'
		<< ", observed_bit_contrast=[" << minimumBitObserved << ',' << maximumBitObserved << ']'
		<< ", observed_phase_amplitude=[" << minimumPhaseAmplitudeObserved << ',' << maximumPhaseAmplitudeObserved << ']';
	localDiagnostics.accepted=projectorPoints.size();
	if (diagnostics) *diagnostics=localDiagnostics;
	if (projectorPoints.size()<12) {
		reason=summary.str()+" (need at least 12)";
		return false;
	}
	reason=summary.str();
	return true;
}
