/*
 * QuadratureDecoder.h
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREENCODER_H_
#define SRC_CLOSEDLOOP_QUADRATUREENCODER_H_

#include <ClosedLoop/SpiEncoder.h>

#if SUPPORT_CLOSED_LOOP

#include <General/FreelistManager.h>
#include <General/NamedEnum.h>

NamedEnum(AttinyProgErrorCode, uint8_t,
	notChecked,
	good,
	spiBusy ,
	cantEnterProgrammingMode ,
	badDeviceId,
	verifyFailed,
	fuseVerifyFailed,
	eraseTimeout,
	writeTimeout,
	fuseWriteTimeout
);

class QuadratureEncoder : public SpiEncoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<QuadratureEncoder>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<QuadratureEncoder>(p); }

	QuadratureEncoder(bool linear) noexcept;
	~QuadratureEncoder() { Disable(); }

	EncoderType GetType() const noexcept override { return (linear) ? EncoderType::linearQuadrature : EncoderType::rotaryQuadrature; }
	void Enable() noexcept override;				// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
	void Disable() noexcept override;				// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	int32_t GetReading() noexcept override;			// Get the 32-bit position
	void AppendDiagnostics(const StringRef& reply) noexcept override;

	void SetReading(int32_t pos) noexcept;			// Set the position. Call this after homing.

	void InitAttiny() noexcept;

	static void TurnAttinyOff() noexcept;
	static AttinyProgErrorCode GetProgramStatus() noexcept { return programStatus; }

private:
	AttinyProgErrorCode CheckProgram() noexcept;	// Check that the decoder is running current firmware, return true if yes
	AttinyProgErrorCode Program() noexcept;			// Update the program, return true if successful

	uint8_t SendSpiQuad(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) noexcept;
	AttinyProgErrorCode SetupForProgramming() noexcept;
	void EndProgramming() noexcept;
	bool WaitUntilAttinyReady() noexcept;
	AttinyProgErrorCode DoVerify() noexcept;

	uint32_t deviceSignature = 0;
	uint16_t counterLow, counterHigh;
	bool linear;									// true if linear, false if rotary

	static AttinyProgErrorCode programStatus;
};

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREENCODER_H_ */