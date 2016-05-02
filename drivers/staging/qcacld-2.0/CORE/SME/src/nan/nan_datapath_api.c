/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: nan_datapath_api.c
 *
 * SME NAN Data path API implementation
 */
#include "smsDebug.h"
#include "sme_Api.h"
#include "smeInside.h"
#include "csrInternal.h"
#include "sme_nan_datapath.h"

/**
 * sme_ndp_initiator_req_handler() - ndp initiator req handler
 * @hal: hal handle
 * @req_params: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
eHalStatus sme_ndp_initiator_req_handler(tHalHandle hal,
			struct ndp_initiator_req *req_params)
{
	eHalStatus status = eHAL_STATUS_SUCCESS;
	tSmeCmd *cmd;
	tpAniSirGlobal mac_ctx = PMAC_STRUCT(hal);

	if (NULL == req_params) {
		smsLog(mac_ctx, LOGE, FL("Invalid req_params"));
		return eHAL_STATUS_INVALID_PARAMETER;
	}

	status = sme_AcquireGlobalLock(&mac_ctx->sme);
	if (eHAL_STATUS_SUCCESS != status) {
		smsLog(mac_ctx, LOGE,
		       FL("SME lock failed, status:%d"), status);
		return status;
	}
	cmd = csrGetCommandBuffer(mac_ctx);
	if (NULL == cmd) {
		sme_ReleaseGlobalLock(&mac_ctx->sme);
		return eHAL_STATUS_RESOURCES;
	}

	cmd->command = eSmeCommandNdpInitiatorRequest;
	cmd->sessionId = (tANI_U8)req_params->vdev_id;
	vos_mem_copy(&cmd->u.initiator_req, req_params,
		     sizeof(struct ndp_initiator_req));
	/* pointers copied as part of above operation are to be overwritten */
	cmd->u.initiator_req.ndp_info.ndp_app_info = NULL;
	cmd->u.initiator_req.ndp_config.ndp_cfg = NULL;

	if (req_params->ndp_info.ndp_app_info_len) {
		cmd->u.initiator_req.ndp_info.ndp_app_info =
			vos_mem_malloc(req_params->ndp_info.ndp_app_info_len);
		if (NULL == cmd->u.initiator_req.ndp_info.ndp_app_info) {
			sme_ReleaseGlobalLock(&mac_ctx->sme);
			return eHAL_STATUS_FAILED_ALLOC;
		}
		vos_mem_copy(cmd->u.initiator_req.ndp_info.ndp_app_info,
			req_params->ndp_info.ndp_app_info,
			req_params->ndp_info.ndp_app_info_len);
	}

	if (req_params->ndp_config.ndp_cfg_len) {
		cmd->u.initiator_req.ndp_config.ndp_cfg =
			vos_mem_malloc(req_params->ndp_config.ndp_cfg_len);
		if (NULL == cmd->u.initiator_req.ndp_config.ndp_cfg) {
			sme_ReleaseGlobalLock(&mac_ctx->sme);
			vos_mem_free(
				cmd->u.initiator_req.ndp_info.ndp_app_info);
			cmd->u.initiator_req.ndp_info.ndp_app_info_len = 0;
			return eHAL_STATUS_FAILED_ALLOC;
		}
		vos_mem_copy(cmd->u.initiator_req.ndp_config.ndp_cfg,
			req_params->ndp_config.ndp_cfg,
			req_params->ndp_config.ndp_cfg_len);
	}

	status = csrQueueSmeCommand(mac_ctx, cmd, TRUE);
	if (eHAL_STATUS_SUCCESS != status) {
		smsLog(mac_ctx, LOGE, FL("SME enqueue failed, status:%d"),
			status);
		vos_mem_free(cmd->u.initiator_req.ndp_info.ndp_app_info);
		vos_mem_free(cmd->u.initiator_req.ndp_config.ndp_cfg);
		cmd->u.initiator_req.ndp_info.ndp_app_info_len = 0;
		cmd->u.initiator_req.ndp_config.ndp_cfg_len = 0;
	}

	sme_ReleaseGlobalLock(&mac_ctx->sme);
	return status;
}

/**
 * sme_ndp_responder_req_handler() - ndp responder request handler
 * @hal: hal handle
 * @req_params: request parameters
 *
 * Return: eHAL_STATUS_SUCCESS on success; error number otherwise
 */
eHalStatus sme_ndp_responder_req_handler(tHalHandle hal,
	struct ndp_responder_req *req_params)
{
	eHalStatus status;
	tSmeCmd *cmd;
	tpAniSirGlobal mac_ctx = PMAC_STRUCT(hal);

	if (NULL == req_params) {
		smsLog(mac_ctx, LOGE, FL("Invalid req_params"));
		return eHAL_STATUS_INVALID_PARAMETER;
	}

	status = sme_AcquireGlobalLock(&mac_ctx->sme);
	if (eHAL_STATUS_SUCCESS != status) {
		smsLog(mac_ctx, LOGE,
			FL("SME lock failed, status:%d"), status);
		return status;
	}
	cmd = csrGetCommandBuffer(mac_ctx);
	if (NULL == cmd) {
		sme_ReleaseGlobalLock(&mac_ctx->sme);
		return eHAL_STATUS_RESOURCES;
	}

	cmd->command = eSmeCommandNdpResponderRequest;
	cmd->sessionId = (tANI_U8)req_params->vdev_id;
	vos_mem_copy(&cmd->u.responder_req, req_params,
			sizeof(*req_params));

	/*
	 * Pointers copied as part of above operation are
	 * to be overwritten
	 */
	cmd->u.responder_req.ndp_info.ndp_app_info = NULL;
	cmd->u.responder_req.ndp_config.ndp_cfg = NULL;

	if (req_params->ndp_info.ndp_app_info_len) {
		cmd->u.responder_req.ndp_info.ndp_app_info =
			vos_mem_malloc(req_params->ndp_info.ndp_app_info_len);
		if (NULL == cmd->u.responder_req.ndp_info.ndp_app_info) {
			sme_ReleaseGlobalLock(&mac_ctx->sme);
			return eHAL_STATUS_FAILED_ALLOC;
		}
		vos_mem_copy(cmd->u.responder_req.ndp_info.ndp_app_info,
			req_params->ndp_info.ndp_app_info,
			req_params->ndp_info.ndp_app_info_len);
	}

	if (req_params->ndp_config.ndp_cfg_len) {
		cmd->u.responder_req.ndp_config.ndp_cfg =
			vos_mem_malloc(req_params->ndp_config.ndp_cfg_len);
		if (NULL == cmd->u.responder_req.ndp_config.ndp_cfg) {
			sme_ReleaseGlobalLock(&mac_ctx->sme);
			vos_mem_free(
				cmd->u.responder_req.ndp_info.ndp_app_info);
			cmd->u.responder_req.ndp_info.ndp_app_info_len = 0;
			return eHAL_STATUS_FAILED_ALLOC;
		}
		vos_mem_copy(cmd->u.responder_req.ndp_config.ndp_cfg,
			req_params->ndp_config.ndp_cfg,
			req_params->ndp_config.ndp_cfg_len);
	}

	status = csrQueueSmeCommand(mac_ctx, cmd, TRUE);
	if (eHAL_STATUS_SUCCESS != status) {
		smsLog(mac_ctx, LOGE,
			FL("SME enqueue failed, status:%d"), status);
		vos_mem_free(cmd->u.responder_req.ndp_info.ndp_app_info);
		vos_mem_free(cmd->u.responder_req.ndp_config.ndp_cfg);
		cmd->u.responder_req.ndp_info.ndp_app_info_len = 0;
		cmd->u.responder_req.ndp_config.ndp_cfg_len = 0;
	}
	sme_ReleaseGlobalLock(&mac_ctx->sme);
	return status;
}

/**
 * sme_ndp_end_req_handler() - ndp end request handler
 * @session_id: session id over which the ndp is being created
 * @req_params: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
VOS_STATUS sme_ndp_end_req_handler(uint32_t session_id,
	struct ndp_end_req *req_params)
{
	return VOS_STATUS_SUCCESS;
}


/**
 * sme_ndp_sched_req_handler() - ndp schedule request handler
 * @session_id: session id over which the ndp is being created
 * @req_params: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
VOS_STATUS sme_ndp_sched_req_handler(uint32_t session_id,
	struct ndp_schedule_update_req *req_params)
{
	return VOS_STATUS_SUCCESS;
}

/**
 * csr_roam_start_ndi() - Start connection for NaN data path
 * @mac_ctx: Global MAC context
 * @session: SME session ID
 * @profile: Configuration profile
 *
 * Return: Success or failure code
 */
VOS_STATUS csr_roam_start_ndi(tpAniSirGlobal mac_ctx, uint32_t session,
			tCsrRoamProfile *profile)
{
	eHalStatus status;
	tBssConfigParam bss_cfg = {0};

	/* Build BSS configuration from profile */
	status = csrRoamPrepareBssConfigFromProfile(mac_ctx, profile,
						    &bss_cfg, NULL);
	if (HAL_STATUS_SUCCESS(status)) {
		mac_ctx->roam.roamSession[session].bssParams.uCfgDot11Mode
			= bss_cfg.uCfgDot11Mode;
		/* Copy profile parameters to PE session */
		csrRoamPrepareBssParams(mac_ctx, session, profile, NULL,
			&bss_cfg, NULL);
		/*
		 * Following routine will eventually call
		 * csrRoamIssueStartBss through csrRoamCcmCfgSetCallback
		 */
		status = csrRoamSetBssConfigCfg(mac_ctx, session, profile, NULL,
					&bss_cfg, NULL, false);
	}

	if (HAL_STATUS_SUCCESS(status))
		smsLog(mac_ctx, LOG1, FL("Profile config is valid"));
	else
		smsLog(mac_ctx, LOGE,
			FL("Profile config invalid. status = 0x%x"), status);

	return status;
}

/**
 * csr_roam_fill_roaminfo_ndp() - fill the ndi create struct inside roam info
 * @mac_ctx: Global MAC context
 * @roam_info: roam info to be updated with ndi create params
 * @roam_result: roam result to update
 * @status_code: status code to update
 * @reason_code: reason code to update
 * @transaction_id: transcation id to update
 *
 * Return: Nothing
 */
void csr_roam_fill_roaminfo_ndp(tpAniSirGlobal mac_ctx,
		tCsrRoamInfo *roam_info,
		eCsrRoamResult roam_result,
		tSirResultCodes status_code,
		uint32_t reason_code,
		uint32_t transaction_id)
{
	struct ndi_create_rsp *rsp_params;

	smsLog(mac_ctx, LOG1,
		FL("reason 0x%x, status 0x%x, transaction_id %d"),
		reason_code, status_code, transaction_id);
	rsp_params = (struct ndi_create_rsp *)
			&roam_info->ndp.ndi_create_params;
	rsp_params->reason = reason_code;
	rsp_params->status = status_code;
	rsp_params->transaction_id = transaction_id;
}

/**
 * csr_roam_save_ndi_connected_info() - Save connected profile parameters
 * @mac_ctx: Global MAC context
 * @session_id: Session ID
 * @roam_profile: Profile given for starting BSS
 * @bssdesc: BSS description from tSirSmeStartBssRsp response
 *
 * Saves NDI profile parameters into session's connected profile.
 *
 * Return: status of operation
 */
void csr_roam_save_ndi_connected_info(tpAniSirGlobal mac_ctx,
				      tANI_U32 session_id,
				      tCsrRoamProfile *roam_profile,
				      tSirBssDescription *bssdesc)
{
	tCsrRoamSession *roam_session = NULL;
	tCsrRoamConnectedProfile *connect_profile = NULL;

	roam_session = CSR_GET_SESSION(mac_ctx, session_id);
	if (NULL == roam_session) {
		VOS_TRACE(VOS_MODULE_ID_SME, VOS_TRACE_LEVEL_ERROR,
			FL("session %d not found"), session_id);
		return;
	}

	connect_profile = &roam_session->connectedProfile;
	vos_mem_set(&roam_session->connectedProfile,
		sizeof(connect_profile), 0);
	connect_profile->AuthType = roam_profile->negotiatedAuthType;
		connect_profile->AuthInfo = roam_profile->AuthType;
	connect_profile->CBMode = roam_profile->CBMode;
	connect_profile->EncryptionType =
		roam_profile->negotiatedUCEncryptionType;
		connect_profile->EncryptionInfo = roam_profile->EncryptionType;
	connect_profile->mcEncryptionType =
		roam_profile->negotiatedMCEncryptionType;
		connect_profile->mcEncryptionInfo =
			roam_profile->mcEncryptionType;
	connect_profile->BSSType = roam_profile->BSSType;
	connect_profile->modifyProfileFields.uapsd_mask =
		roam_profile->uapsd_mask;
	connect_profile->operationChannel = bssdesc->channelId;
	connect_profile->beaconInterval = 0;
	vos_mem_copy(&connect_profile->Keys, &roam_profile->Keys,
		sizeof(roam_profile->Keys));
	csrGetBssIdBssDesc(mac_ctx, bssdesc, &connect_profile->bssid);
	connect_profile->SSID.length = 0;
	csrFreeConnectBssDesc(mac_ctx, session_id);
	connect_profile->qap = FALSE;
	connect_profile->qosConnection = FALSE;
}

/**
 * csr_roam_update_ndp_return_params() - updates ndp return parameters
 * @mac_ctx: MAC context handle
 * @result: result of the roaming command
 * @roam_status: roam status returned to the roam command initiator
 * @roam_result: roam result returned to the roam command initiator
 * @rinfo: Roam info context
 *
 * Results: None
 */
void csr_roam_update_ndp_return_params(tpAniSirGlobal mac_ctx,
					uint32_t result,
					uint32_t *roam_status,
					uint32_t *roam_result,
					void *rinfo)
{
	tCsrRoamInfo *roam_info = (tCsrRoamInfo *)rinfo;

	switch (result) {
	case eCsrStartBssSuccess:
	case eCsrStartBssFailure:
		*roam_status = eCSR_ROAM_NDP_STATUS_UPDATE;
		*roam_result = eCSR_ROAM_RESULT_NDP_CREATE_RSP;
		break;
	case eCsrStopBssSuccess:
		*roam_status = eCSR_ROAM_NDP_STATUS_UPDATE;
		*roam_result = eCSR_ROAM_RESULT_NDP_DELETE_RSP;
		roam_info->ndp.ndi_delete_params.status = VOS_STATUS_SUCCESS;
		break;
	case eCsrStopBssFailure:
		*roam_status = eCSR_ROAM_NDP_STATUS_UPDATE;
		*roam_result = eCSR_ROAM_RESULT_NDP_DELETE_RSP;
		roam_info->ndp.ndi_delete_params.status = VOS_STATUS_E_FAILURE;
		break;
	default:
		smsLog(mac_ctx, LOGE,
			FL("Invalid CSR Roam result code: %d"), result);
		break;
	}
}

/**
 * csr_process_ndp_initiator_request() - process ndp initiator request
 * @mac_ctx: Global MAC context
 * @cmd: ndp initiator sme cmd
 *
 * Return: status of operation
 */
eHalStatus csr_process_ndp_initiator_request(tpAniSirGlobal mac_ctx,
					     tSmeCmd *cmd)
{
	struct sir_sme_ndp_initiator_req *lim_msg;
	uint16_t msg_len;
	uint8_t *self_mac_addr = NULL;
	struct ndp_initiator_req *req;

	if (NULL == cmd) {
		smsLog(mac_ctx, LOGE, FL("Invalid req_params"));
		return eHAL_STATUS_INVALID_PARAMETER;
	}
	req = &cmd->u.initiator_req;

	msg_len = sizeof(*lim_msg);
	lim_msg = vos_mem_malloc(msg_len);
	if (NULL == lim_msg)
		return eHAL_STATUS_FAILED_ALLOC;

	vos_mem_set(lim_msg, msg_len, 0);
	lim_msg->msg_type =
		pal_cpu_to_be16((uint16_t)eWNI_SME_NDP_INITIATOR_REQ);
	lim_msg->msg_len = pal_cpu_to_be16(msg_len);
	/*
	 * following is being copied from p_cmd->u.initiator_req,
	 * no need to perform deep copy, as we are going to use memory
	 * allocated at SME in p_cmd->u.initiator_req and pass it all the way
	 * to WMA.
	 */
	vos_mem_copy(&lim_msg->req, req, sizeof(struct ndp_initiator_req));

	self_mac_addr = lim_msg->req.self_ndi_mac_addr.bytes;
	smsLog(mac_ctx, LOG1, FL("selfMac = "MAC_ADDRESS_STR),
		MAC_ADDR_ARRAY(self_mac_addr));

	return palSendMBMessage(mac_ctx->hHdd, lim_msg);
}

/**
 * csr_process_ndp_responder_request() - ndp responder req
 * @mac_ctx: Global MAC context
 * @cmd: Cmd sent to SME
 *
 * Return: Success or failure code
 */
eHalStatus csr_process_ndp_responder_request(tpAniSirGlobal mac_ctx,
							tSmeCmd *cmd)
{
	struct sir_sme_ndp_responder_req *lim_msg;
	uint16_t msg_len;
	eHalStatus status;

	if (!cmd) {
		smsLog(mac_ctx, LOGE, FL("Invalid req_params"));
		return eHAL_STATUS_INVALID_PARAMETER;
	}

	msg_len  = sizeof(*lim_msg);
	lim_msg = vos_mem_malloc(msg_len);
	if (!lim_msg) {
		smsLog(mac_ctx, LOGE, FL("Mem alloc fail"));
		status = eHAL_STATUS_FAILED_ALLOC;
		goto free_config;
	}

	vos_mem_set(lim_msg, msg_len, 0);
	lim_msg->msg_type =
		pal_cpu_to_be16((uint16_t)eWNI_SME_NDP_RESPONDER_REQ);
	lim_msg->msg_len = pal_cpu_to_be16(msg_len);
	/*
	 * following is being copied from p_cmd->u.initiator_req,
	 * no need to perform deep copy, as we are going to use memory
	 * allocated at SME in p_cmd->u.initiator_req and pass it all the way
	 * to WMA.
	 */
	vos_mem_copy(&lim_msg->req, &cmd->u.responder_req,
			sizeof(struct ndp_responder_req));

	smsLog(mac_ctx, LOG1,
		FL("vdev_id %d ndp_rsp = %d Instance id %d"),
		lim_msg->req.vdev_id,
		lim_msg->req.ndp_rsp,
		lim_msg->req.ndp_instance_id);

	status = palSendMBMessage(mac_ctx->hHdd, lim_msg);

free_config:
	if (!HAL_STATUS_SUCCESS(status)) {
		/*
		 * If fail, free up the ndp_cfg and ndp_app_info
		 * allocated in sme.
		 */
		vos_mem_free(cmd->u.responder_req.ndp_info.ndp_app_info);
		vos_mem_free(cmd->u.responder_req.ndp_config.ndp_cfg);
		cmd->u.responder_req.ndp_info.ndp_app_info_len = 0;
		cmd->u.responder_req.ndp_config.ndp_cfg_len = 0;
		cmd->u.responder_req.ndp_config.ndp_cfg = NULL;
		cmd->u.responder_req.ndp_info.ndp_app_info = NULL;
	}
	return status;
}

/**
 * sme_ndp_msg_processor() - message processor for ndp/ndi north-bound SME msg.
 * @mac_ctx: Global MAC context
 * @msg: ndp/ndi SME message
 *
 * This function will further call csrRoamCallCallback with appropriate
 * roam_status and roam_result thus allowing hdd to correctly identify NDP/NDI
 * response.
 *
 * Return: nothing
 */
void sme_ndp_msg_processor(tpAniSirGlobal mac_ctx, vos_msg_t *msg)
{
	tCsrRoamInfo roam_info = {0};
	eCsrRoamResult result;
	uint32_t session_id;
	tListElem *entry = NULL;
	tSmeCmd *cmd = NULL;
	bool release_active_cmd = false;
	eSmeCommandType cmd_to_rel = eSmeNoCommand;
	bool send_to_user = true;

	switch (msg->type) {
	case eWNI_SME_NDP_CONFIRM_IND: {
		result = eCSR_ROAM_RESULT_NDP_CONFIRM_IND;
		/* copy msg from msg body to roam info passed to callback */
		vos_mem_copy(&roam_info.ndp.ndp_confirm_params, msg->bodyptr,
			sizeof(roam_info.ndp.ndp_confirm_params));
		session_id = roam_info.ndp.ndp_confirm_params.vdev_id;
		break;
	}
	case eWNI_SME_NDP_INITIATOR_RSP: {
		if (true == msg->bodyval) {
			/* rsp was locally generated, do not send to HDD */
			send_to_user = false;
		} else {
			result = eCSR_ROAM_RESULT_NDP_INITIATOR_RSP;
			vos_mem_copy(&roam_info.ndp.ndp_init_rsp_params,
				     msg->bodyptr,
				     sizeof(roam_info.ndp.ndp_init_rsp_params));
			session_id = roam_info.ndp.ndp_init_rsp_params.vdev_id;
		}
		release_active_cmd = true;
		cmd_to_rel = eSmeCommandNdpInitiatorRequest;
		break;
	}
	case eWNI_SME_NDP_NEW_PEER_IND: {
		result = eCSR_ROAM_RESULT_NDP_NEW_PEER_IND;
		/* copy msg from msg body to roam info passed to callback */
		vos_mem_copy(&roam_info.ndp.ndp_peer_ind_params,
			     msg->bodyptr,
			     sizeof(roam_info.ndp.ndp_peer_ind_params));
		session_id = roam_info.ndp.ndp_peer_ind_params.session_id;
		break;
	}
	case eWNI_SME_NDP_INDICATION:
		result = eCSR_ROAM_RESULT_NDP_INDICATION;
		/* copy msg from msg body to roam info passed to callback */
		vos_mem_copy(&roam_info.ndp.ndp_indication_params,
			msg->bodyptr, sizeof(struct ndp_indication_event));
		session_id = roam_info.ndp.ndp_indication_params.vdev_id;
		break;
	case eWNI_SME_NDP_RESPONDER_RSP:
		if (true == msg->bodyval) {
			/* rsp was locally generated, do not send to HDD */
			send_to_user = false;
		} else {
			result = eCSR_ROAM_RESULT_NDP_RESPONDER_RSP;
			/*
			 * Copy msg from msg body to roam info passed to
			 * callback
			 */
			vos_mem_copy(&roam_info.ndp.ndp_responder_rsp_params,
				msg->bodyptr,
				sizeof(struct ndp_responder_rsp_event));
			session_id =
				roam_info.ndp.ndp_responder_rsp_params.vdev_id;
		}
		release_active_cmd = true;
		cmd_to_rel = eSmeCommandNdpResponderRequest;
		break;
	default:
		smsLog(mac_ctx, LOGE, FL("Unhandled NDP rsp"));
		vos_mem_free(msg->bodyptr);
		return;
	}

	if (true == send_to_user) {
		csrRoamCallCallback(mac_ctx, session_id, &roam_info, 0,
				    eCSR_ROAM_NDP_STATUS_UPDATE, result);
	}

	/* free ndp_cfg and ndp_app_info if required
	 * For some commands this info may be needed in HDD
	 * so free them after roam callback.
	 */
	switch (msg->type) {
	case eWNI_SME_NDP_INITIATOR_RSP:
		entry = csrLLPeekHead(&mac_ctx->sme.smeCmdActiveList,
				LL_ACCESS_LOCK);
		if (entry != NULL) {
			cmd = GET_BASE_ADDR(entry, tSmeCmd, Link);
			if (eSmeCommandNdpInitiatorRequest == cmd->command) {
				vos_mem_free(
					cmd->u.initiator_req.
					ndp_config.ndp_cfg);
				vos_mem_free(
					cmd->u.initiator_req.
					ndp_info.ndp_app_info);
			}
		}
		break;
	case eWNI_SME_NDP_RESPONDER_RSP:
		entry = csrLLPeekHead(&mac_ctx->sme.smeCmdActiveList,
					LL_ACCESS_LOCK);
		if (entry != NULL) {
			cmd = GET_BASE_ADDR(entry, tSmeCmd, Link);
			if (eSmeCommandNdpResponderRequest == cmd->command) {
				vos_mem_free(
					cmd->u.responder_req.
					ndp_config.ndp_cfg);
				vos_mem_free(
					cmd->u.responder_req.
					ndp_info.ndp_app_info);
			}
		}
		break;
	case eWNI_SME_NDP_INDICATION:
		vos_mem_free(
			roam_info.ndp.ndp_indication_params.ndp_config.ndp_cfg);
		vos_mem_free(
			roam_info.ndp.ndp_indication_params.
			ndp_info.ndp_app_info);
		break;
	default:
		break;
	}
	vos_mem_free(msg->bodyptr);
	if (release_active_cmd == false)
		return;

	entry = csrLLPeekHead(&mac_ctx->sme.smeCmdActiveList, LL_ACCESS_LOCK);
	if (entry == NULL)
		return;

	cmd = GET_BASE_ADDR(entry, tSmeCmd, Link);
	if (cmd_to_rel == cmd->command) {
		/* Now put this cmd back on the avilable command list */
		if (csrLLRemoveEntry(&mac_ctx->sme.smeCmdActiveList,
				     entry, LL_ACCESS_LOCK))
			smeReleaseCommand(mac_ctx, cmd);
		smeProcessPendingQueue(mac_ctx);
	}
}
