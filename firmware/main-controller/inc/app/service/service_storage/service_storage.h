/**
 * @file service_storage.h
 * @brief Upper storage service wrapper for internal flash access.
 */

#ifndef MAD_CIRCUITS_APP_SERVICE_STORAGE_H
#define MAD_CIRCUITS_APP_SERVICE_STORAGE_H

#include "../../../../config/app/service/service_storage/service_storage_cfg.h"
#include "Ifx_Types.h"

/**
 * @brief Initialize storage service runtime buffers and job state.
 * @param[in] storage_id Storage service instance id.
 * @return void
 */
void service_storage_init(service_storage_id_t storage_id);

/**
 * @brief Initialize all storage service instances.
 * @param[in] void No parameter.
 * @return void
 */
void service_storage_init_all(void);

/**
 * @brief Start or stop one asynchronous storage session.
 * @note Do not mix asynchronous APIs with synchronous APIs in one init cycle.
 *       Reinitialize the storage service before switching modes.
 * @note Call with SERVICE_STORAGE_JOB_WRITE or SERVICE_STORAGE_JOB_READ only when
 *       the current job is NONE.
 * @note For asynchronous write, calling with SERVICE_STORAGE_JOB_NONE only submits
 *       a close request for the current write session. The write job does not end
 *       immediately. Keep calling service_storage_runAsync(...) until the pending
 *       buffered data is fully drained and the internal job state returns to NONE.
 * @note For asynchronous read, calling with SERVICE_STORAGE_JOB_NONE ends the read
 *       session immediately.
 * @param[in] storage_id Storage service instance id.
 * @param[in] job_type Requested storage job type.
 * @return void
 */
void service_storage_setJobTypeAsync(service_storage_id_t storage_id, service_storage_job_type_t job_type);

/**
 * @brief Execute one background storage service step.
 * @note Only valid for asynchronous buffer-based access.
 * @note This is the only execution path that actually drains asynchronous write
 *       buffers into flash. Keep scheduling it while an asynchronous write session
 *       is active, including after requesting SERVICE_STORAGE_JOB_NONE to close.
 * @note For asynchronous read, this API preloads the background buffer for the
 *       next read window.
 * @param[in] storage_id Storage service instance id.
 * @return void
 */
void service_storage_runAsync(service_storage_id_t storage_id);

/**
 * @brief Directly write flash in synchronous mode without service buffers.
 * @note Do not mix synchronous APIs with asynchronous APIs in one init cycle.
 *       Reinitialize the storage service before switching modes.
 * @note group_offset and length must align with IFXFLASH_PFLASH_PAGE_LENGTH.
 * @param[in] storage_id Storage service instance id.
 * @param[in] group_offset Group-relative write offset, unit: byte.
 * @param[in] data Source buffer pointer.
 * @param[in] length Write length, unit: byte.
 * @return Actual written length, unit: byte.
 */
uint32 service_storage_writeSync(service_storage_id_t storage_id,
                                 uint32 group_offset,
                                 const uint8* data,
                                 uint32 length);

/**
 * @brief Directly read flash in synchronous mode without service buffers.
 * @note Do not mix synchronous APIs with asynchronous APIs in one init cycle.
 *       Reinitialize the storage service before switching modes.
 * @param[in] storage_id Storage service instance id.
 * @param[in] group_offset Group-relative read offset, unit: byte.
 * @param[out] data Destination buffer pointer.
 * @param[in] length Requested read length, unit: byte.
 * @return Actual read length, unit: byte.
 */
uint32 service_storage_readSync(service_storage_id_t storage_id,
                                uint32 group_offset,
                                uint8* data,
                                uint32 length);

/**
 * @brief Directly erase the whole flash group of one storage module.
 * @note This API directly erases flash and then resets service_storage state.
 *       It can be used regardless of current synchronous or asynchronous mode.
 * @param[in] storage_id Storage service instance id.
 * @return TRUE if erase request is accepted, otherwise FALSE.
 */
boolean service_storage_erase(service_storage_id_t storage_id);

/**
 * @brief Append continuous data into storage write buffer.
 * @note Do not use this asynchronous API together with synchronous APIs
 *       in one init cycle.
 * @note Call service_storage_setJobTypeAsync(..., SERVICE_STORAGE_JOB_WRITE)
 *       before the first write of one session.
 * @note After all data has been appended, call
 *       service_storage_setJobTypeAsync(..., SERVICE_STORAGE_JOB_NONE) to request
 *       write-session close, then keep calling service_storage_runAsync(...) until
 *       the session is fully drained. Do not treat the close request itself as
 *       flash-write completion.
 * @note Once write-session close has been requested, further writeAsync calls are
 *       rejected until the current asynchronous write session has finished.
 * @param[in] storage_id Storage service instance id.
 * @param[in] data Source buffer pointer.
 * @param[in] length Write length, unit: byte.
 * @return Actual buffered length, unit: byte. Returns 0 if the session is not in
 *         WRITE state, the input is invalid, or the write-close phase has started.
 */
uint32 service_storage_writeAsync(service_storage_id_t storage_id,
                                  const uint8* data,
                                  uint32 length);

/**
 * @brief Read continuous data from storage read buffer by group offset.
 * @note Do not use this asynchronous API together with synchronous APIs
 *       in one init cycle.
 * @note Call service_storage_setJobTypeAsync(..., SERVICE_STORAGE_JOB_READ)
 *       before the first read of one session. The first read window is loaded
 *       immediately during read-session start.
 * @note Keep calling service_storage_runAsync(...) while reading so the next
 *       background read window can be prefetched.
 * @param[in] storage_id Storage service instance id.
 * @param[in] group_offset Group-relative read offset, unit: byte.
 * @param[out] data Destination buffer pointer.
 * @param[in] length Requested read length, unit: byte.
 * @return Actual read length, unit: byte.
 */
uint32 service_storage_readAsync(service_storage_id_t storage_id,
                                 uint32 group_offset,
                                 uint8* data,
                                 uint32 length);

#endif
