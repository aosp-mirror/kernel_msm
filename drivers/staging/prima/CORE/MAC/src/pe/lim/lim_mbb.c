/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "aniGlobal.h"
#include "limTypes.h"
#include "limUtils.h"
#include "limFT.h"
#include "limSendMessages.h"
#include "limAssocUtils.h"
#include "limSerDesUtils.h"
#include "limSmeReqUtils.h"
#include "limAdmitControl.h"
#include "sirApi.h"
#include "rrmApi.h"
#include "wlan_qct_tl.h"

#define PREAUTH_REASSOC_TIMEOUT 500

void lim_cleanup_connected_ap(tpAniSirGlobal mac, tpDphHashNode sta_ds,
     tpPESession session_entry);

/**
 * lim_post_pre_auth_reassoc_rsp() -Posts preauth_reassoc response to SME
 * @mac: MAC context
 * @status: status
 * @session_entry: session entry
 * @reason: indicates which type of clean up needs to be performed
 *
 * This function process preauth_reassoc response to SME
 */
void lim_post_pre_auth_reassoc_rsp(tpAniSirGlobal mac,
     tSirRetStatus status, tpPESession session_entry,
     enum sir_roam_cleanup_type reason)
{
    tpSirFTPreAuthRsp pre_auth_rsp;
    tSirMsgQ mmh_msg;
    tANI_U16 rsp_len = sizeof(tSirFTPreAuthRsp);
    tpPESession session_entry_con_ap;
    tpDphHashNode sta_ds = NULL;

    pre_auth_rsp = (tpSirFTPreAuthRsp)vos_mem_malloc(rsp_len);
    if (NULL == pre_auth_rsp) {
        limLog(mac, LOGE, FL("Failed to allocate memory"));
        return;
    }

    limLog(mac, LOG1, FL("reason %d"), reason);

    vos_mem_zero(pre_auth_rsp, rsp_len);
    pre_auth_rsp->messageType = eWNI_SME_MBB_PRE_AUTH_REASSOC_RSP;
    pre_auth_rsp->length = (tANI_U16)rsp_len;
    pre_auth_rsp->status = status;
    pre_auth_rsp->reason = reason;

    if (session_entry)
        pre_auth_rsp->smeSessionId = session_entry->smeSessionId;

    /* The bssid of the AP we are sending Auth1 to. */
    if (mac->ft.ftPEContext.pFTPreAuthReq)
        sirCopyMacAddr(pre_auth_rsp->preAuthbssId,
                       mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId);

    if (status != eSIR_SUCCESS) {
        limLog(mac, LOG1, FL("Pre-Auth Failed, Cleanup!"));
        limLog(mac, LOG1, FL("flushing cached packets"));
        WLANTL_PreAssocForward(false);

        /*
        * If reason is full clean up, add sme session id that
        * will be useful in CSR during cleanup.
        */
        if (reason == SIR_MBB_DISCONNECTED) {
            session_entry_con_ap =
               (tpPESession)mac->ft.ftPEContext.psavedsessionEntry;
            pre_auth_rsp->smeSessionId =
                  session_entry_con_ap->smeSessionId;
        }
        limFTCleanup(mac);
    }

    if (status == eSIR_SUCCESS) {
        limLog(mac, LOG1, FL("Success"));

        rsp_len = session_entry->assocReqLen + session_entry->assocRspLen +
                  session_entry->bcnLen;

        pre_auth_rsp->roam_info = vos_mem_malloc(sizeof(tCsrRoamInfo));
        if (pre_auth_rsp->roam_info == NULL) {
            limLog(mac, LOGE,
                   FL("Failed to allocate memory for roam info"));
            return;
        }
        vos_mem_set(pre_auth_rsp->roam_info, sizeof(tCsrRoamInfo), 0);

        pre_auth_rsp->roam_info->pbFrames = vos_mem_malloc(rsp_len);
        if (pre_auth_rsp->roam_info->pbFrames == NULL) {
            limLog(mac, LOGE,
                   FL("Failed to allocate memory for roam info frames"));
            return;
        }
        vos_mem_set(pre_auth_rsp->roam_info->pbFrames, rsp_len, 0);


        pre_auth_rsp->length += rsp_len + sizeof(tCsrRoamInfo);

        session_entry_con_ap =
               (tpPESession)mac->ft.ftPEContext.psavedsessionEntry;
            pre_auth_rsp->smeSessionId = session_entry_con_ap->smeSessionId;

        if(session_entry->beacon != NULL) {
           pre_auth_rsp->roam_info->nBeaconLength = session_entry->bcnLen;
           vos_mem_copy(pre_auth_rsp->roam_info->pbFrames,
                        session_entry->beacon,
                        session_entry->bcnLen);
           limLog(mac, LOG1, FL("Beacon len %d"), session_entry->bcnLen);

           vos_mem_free(session_entry->beacon);
           session_entry->beacon = NULL;
        }

        if(session_entry->assocReq != NULL) {
           pre_auth_rsp->roam_info->nAssocReqLength =
                                      session_entry->assocReqLen;
           vos_mem_copy(pre_auth_rsp->roam_info->pbFrames +
                        session_entry->bcnLen,
                        session_entry->assocReq,
                        session_entry->assocReqLen);

           vos_mem_free(session_entry->assocReq);
           session_entry->assocReq = NULL;

           limLog(mac, LOG1, FL("AssocReq len %d"), session_entry->assocReqLen);
        }

        if(session_entry->assocRsp != NULL) {
           pre_auth_rsp->roam_info->nAssocRspLength =
                                   session_entry->assocRspLen;
           vos_mem_copy(pre_auth_rsp->roam_info->pbFrames +
                        session_entry->bcnLen + session_entry->assocReqLen,
                        session_entry->assocRsp,
                        session_entry->assocRspLen);

           vos_mem_free(session_entry->assocRsp);
           session_entry->assocRsp = NULL;

           limLog(mac, LOG1, FL("AssocRsp len %d"), session_entry->assocRspLen);
        }

        sta_ds = dphGetHashEntry(mac, DPH_STA_HASH_INDEX_PEER,
                                   &session_entry->dph.dphHashTable);
        if(NULL == sta_ds) {
           limLog(mac, LOGE,
                  FL("Unable to get the DPH Hash Entry for AID - %d"),
                  DPH_STA_HASH_INDEX_PEER);
           return;
        }

        pre_auth_rsp->roam_info->staId = sta_ds->staIndex;
        pre_auth_rsp->roam_info->ucastSig = sta_ds->ucUcastSig;
        pre_auth_rsp->roam_info->bcastSig = sta_ds->ucBcastSig;
        pre_auth_rsp->roam_info->maxRateFlags =
                         limGetMaxRateFlags(sta_ds, session_entry);
    }

    mmh_msg.type = pre_auth_rsp->messageType;
    mmh_msg.bodyptr = pre_auth_rsp;
    mmh_msg.bodyval = 0;

    limLog(mac, LOG1,
           FL("Posted Auth Rsp to SME with status of 0x%x"), status);

    limSysProcessMmhMsgApi(mac, &mmh_msg, ePROT);
}

/*
 * lim_reassoc_fail_cleanup() -handles cleanup during reassoc failure
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 *
 * This function handles cleanup during reassoc failure
 */
void lim_reassoc_fail_cleanup(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data)
{
    tpPESession session_entry;

    session_entry = (tpPESession)data;

    if (!mac->ft.ftPEContext.pFTPreAuthReq) {
        limLog(mac, LOGE, FL("pFTPreAuthReq is NULL"));
        return;
    }

    if (dphDeleteHashEntry(mac,
               mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId,
               DPH_STA_HASH_INDEX_PEER,
               &session_entry->dph.dphHashTable) != eSIR_SUCCESS) {
        limLog(mac, LOGE, FL("error deleting hash entry"));
    }

    /* Delete session as session was created during preauth success */
    peDeleteSession(mac, session_entry);

    /* Add bss parameter cleanup happens as part of this processing*/
    if ((status == eHAL_STATUS_MBB_DEL_BSS_FAIL) ||
        (status == eHAL_STATUS_INVALID_PARAMETER) ||
        (status == eHAL_STATUS_MBB_ADD_BSS_FAIL))
        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                           SIR_MBB_DISCONNECTED);
     else
        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                             SIR_MBB_CONNECTED);
}

/*
 * lim_perform_post_reassoc_mbb_channel_change() -invokes resume callback
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 * @session_entry: session entry
 *
 * This function invokes resume callback
 */
void lim_perform_post_reassoc_mbb_channel_change(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data, tpPESession session_entry)
{
    tpPESession session_entry_con_ap;
    tANI_U8 session_id;

    session_entry_con_ap = peFindSessionByBssid(mac,
                         mac->ft.ftPEContext.pFTPreAuthReq->currbssId,
                         &session_id);
    if (session_entry_con_ap == NULL) {
        limLog(mac, LOGE,
               FL("session does not exist for given BSSID" MAC_ADDRESS_STR),
               MAC_ADDR_ARRAY(mac->ft.ftPEContext.pFTPreAuthReq->currbssId));

        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                             SIR_MBB_CONNECTED);
        return;
    }

    peSetResumeChannel(mac, session_entry_con_ap->currentOperChannel, 0);
    limResumeLink(mac, lim_reassoc_fail_cleanup,
                                (tANI_U32 *)session_entry);
}

/*
 * lim_handle_reassoc_mbb_fail() -handles reassoc failure
 * @mac: MAC context
 * @session_entry: session entry
 *
 * This function handles reassoc failure
 */
void lim_handle_reassoc_mbb_fail(tpAniSirGlobal mac,
     tpPESession session_entry)
{
    tpPESession session_entry_con_ap;
    tANI_U8 session_id;

    session_entry_con_ap = peFindSessionByBssid(mac,
                         mac->ft.ftPEContext.pFTPreAuthReq->currbssId,
                         &session_id);
    if (session_entry_con_ap == NULL) {
        limLog(mac, LOGE,
               FL("session does not exist for given BSSID" MAC_ADDRESS_STR),
               MAC_ADDR_ARRAY(mac->ft.ftPEContext.pFTPreAuthReq->currbssId));

        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                             SIR_MBB_CONNECTED);
        return;
    }

    limLog(mac, LOG1, FL("currentOperChannel %d preAuthchannelNum %d"),
           session_entry_con_ap->currentOperChannel,
           mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum);

    /* Restore set link to post assoc for currently connected AP */
    if (limSetLinkState(mac, eSIR_LINK_POSTASSOC_STATE,
                        session_entry_con_ap->bssId,
                        session_entry_con_ap->selfMacAddr,
                        NULL, NULL) != eSIR_SUCCESS) {
        limLog(mac, LOGE, FL("Set link state to POSTASSOC failed"));

        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                             SIR_MBB_CONNECTED);
        return;
    }

    /* Change channel if required as channel might be changed during preauth */
    if (session_entry_con_ap->currentOperChannel !=
            mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum) {
        limChangeChannelWithCallback(mac,
                       session_entry_con_ap->currentOperChannel,
           lim_perform_post_reassoc_mbb_channel_change, NULL, session_entry);
    } else {
       /*
        * Link needs to be resumed as link was suspended
        * for same channel during preauth.
        */
       peSetResumeChannel(mac, session_entry_con_ap->currentOperChannel, 0);
       limResumeLink(mac, lim_reassoc_fail_cleanup,
                     (tANI_U32 *)session_entry);
    }
}

/*
 * lim_preauth_fail_cleanup() -handles cleanup during preauth failure
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 *
 * This function handles cleanup during reassoc failure
 */
void lim_preauth_fail_cleanup(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data)
{
    tpPESession session_entry;

    session_entry = (tpPESession)data;

    lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                            SIR_MBB_CONNECTED);
}

/*
 * lim_perform_preauth_mbb_fail_channel_change() -invokes resume callback
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 * @session_entry: session entry
 *
 * This function invokes resume callback
 */
void lim_perform_preauth_mbb_fail_channel_change(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data, tpPESession session_entry)
{
    peSetResumeChannel(mac, session_entry->currentOperChannel, 0);
    limResumeLink(mac, lim_preauth_fail_cleanup,
                                (tANI_U32 *)session_entry);
}

/*
 * lim_handle_preauth_mbb_fail() -handles preauth failure
 * @mac: MAC context
 * @session_entry: session entry
 *
 * This function handles reassoc failure
 */
void lim_handle_preauth_mbb_fail(tpAniSirGlobal mac,
     tpPESession session_entry)
{
    limLog(mac, LOG1, FL("currentOperChannel %d preAuthchannelNum %d"),
           session_entry->currentOperChannel,
           mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum);

    /* Restore set link to post assoc for currently connected AP */
    if (limSetLinkState(mac, eSIR_LINK_POSTASSOC_STATE,
                        session_entry->bssId,
                        session_entry->selfMacAddr,
                        NULL, NULL) != eSIR_SUCCESS) {
        limLog(mac, LOGE, FL("Set link state to POSTASSOC failed"));

        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                             SIR_MBB_CONNECTED);
        return;
    }

    /* Change channel if required as channel might be changed during preauth */
    if (session_entry->currentOperChannel !=
            mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum) {
        limChangeChannelWithCallback(mac,
                       session_entry->currentOperChannel,
           lim_perform_preauth_mbb_fail_channel_change, NULL, session_entry);
    } else {
       /*
        * Link needs to be resumed as link was suspended
        * for same channel during preauth.
        */
       peSetResumeChannel(mac, session_entry->currentOperChannel, 0);
       limResumeLink(mac, lim_preauth_fail_cleanup,
                     (tANI_U32 *)session_entry);
    }
}


/*
 * lim_del_sta_mbb() -performs del sta
 * @mac: MAC context
 * @sta_ds_connected_ap: station entry of connected AP
 * @resp_reqd: indicates whether response is required or not
 * @session_entry_connected_ap: session entry of connected AP
 *
 * This function performs del sta
 */
tSirRetStatus lim_del_sta_mbb(tpAniSirGlobal mac,
    tpDphHashNode sta_ds_connected_ap,
    tANI_BOOLEAN resp_reqd,
    tpPESession session_entry_connected_ap)
{
    tpDeleteStaParams del_sta_params;
    tSirMsgQ msg;
    tSirRetStatus ret_code;

    del_sta_params = vos_mem_malloc(sizeof(*del_sta_params));
    if (NULL == del_sta_params) {
        limLog(mac, LOGE, FL("Unable to allocate memory during DEL_STA" ));
        return eSIR_MEM_ALLOC_FAILED;
    }
    vos_mem_zero(del_sta_params, sizeof(*del_sta_params));

    del_sta_params->sessionId = session_entry_connected_ap->peSessionId;
    del_sta_params->status  = eHAL_STATUS_SUCCESS;

#ifdef FEATURE_WLAN_TDLS
    if(((eLIM_STA_ROLE == GET_LIM_SYSTEM_ROLE(session_entry_connected_ap)) &&
        (sta_ds_connected_ap->staType !=  STA_ENTRY_TDLS_PEER)) ||
        (eLIM_BT_AMP_STA_ROLE ==
                         GET_LIM_SYSTEM_ROLE(session_entry_connected_ap)))
#else
    if((eLIM_STA_ROLE == GET_LIM_SYSTEM_ROLE(session_entry_connected_ap)) ||
              (eLIM_BT_AMP_STA_ROLE ==
                       GET_LIM_SYSTEM_ROLE(session_entry_connected_ap)))
#endif
      del_sta_params->staIdx = session_entry_connected_ap->staId;
    else
      del_sta_params->staIdx = sta_ds_connected_ap->staIndex;

    del_sta_params->assocId = sta_ds_connected_ap->assocId;
    del_sta_params->respReqd = resp_reqd;

    /* Change Mlm state of connected AP to Del sta rsp state */
    session_entry_connected_ap->limMlmState = eLIM_MLM_WT_DEL_STA_RSP_STATE;
    MTRACE(macTrace(mac, TRACE_CODE_MLM_STATE,
      session_entry_connected_ap->peSessionId, eLIM_MLM_WT_DEL_STA_RSP_STATE));

    msg.type = WDA_DELETE_STA_REQ;
    msg.reserved = 0;
    msg.bodyptr = del_sta_params;
    msg.bodyval = 0;

    limLog(mac, LOG1,
           FL("sessionId %d staIdx: %d assocId: %d for "MAC_ADDRESS_STR),
           del_sta_params->sessionId, del_sta_params->staIdx,
           del_sta_params->assocId,
           MAC_ADDR_ARRAY(sta_ds_connected_ap->staAddr));

    MTRACE(macTraceMsgTx(mac, session_entry_connected_ap->peSessionId,
                                                               msg.type));

    ret_code = wdaPostCtrlMsg(mac, &msg);
    if( eSIR_SUCCESS != ret_code) {
        if(resp_reqd)
           SET_LIM_PROCESS_DEFD_MESGS(mac, true);
        limLog(mac, LOGE,
               FL("Posting DELETE_STA_REQ failed, reason=%X"), ret_code);
        vos_mem_free(del_sta_params);
    }

    return ret_code;
}

/*
 * lim_del_bss_mbb() -performs del bss of connected AP
 * @mac: MAC context
 * @sta_ds: station entry
 * @bss_idx:BSS index
 * @session_entry: session entry
 *
 * This function performs del bss of connected AP
 */
tSirRetStatus lim_del_bss_mbb(tpAniSirGlobal mac, tpDphHashNode sta_ds,
    tANI_U16 bss_idx,tpPESession session_entry)
{
    tpDeleteBssParams delbss_params = NULL;
    tSirMsgQ msg;
    tSirRetStatus ret_code = eSIR_SUCCESS;

    delbss_params = vos_mem_malloc(sizeof(tDeleteBssParams));
    if (NULL == delbss_params) {
        limLog(mac, LOGE,
               FL("Unable to allocate memory during del bss" ));
        return eSIR_MEM_ALLOC_FAILED;
    }
    vos_mem_set((tANI_U8 *) delbss_params, sizeof(tDeleteBssParams), 0);

    delbss_params->sessionId = session_entry->peSessionId;

    if (sta_ds != NULL) {
        delbss_params->bssIdx = sta_ds->bssId;
        sta_ds->valid = 0;
        sta_ds->mlmStaContext.mlmState = eLIM_MLM_WT_DEL_BSS_RSP_STATE;
    }
    else
        delbss_params->bssIdx = bss_idx;

    session_entry->limMlmState = eLIM_MLM_WT_DEL_BSS_RSP_STATE;
    MTRACE(macTrace(mac, TRACE_CODE_MLM_STATE,
          session_entry->peSessionId, eLIM_MLM_WT_DEL_BSS_RSP_STATE));

    delbss_params->status= eHAL_STATUS_SUCCESS;
    delbss_params->respReqd = 1;

    limLog(mac, LOG1, FL("Sessionid %d bss idx: %x BSSID:" MAC_ADDRESS_STR),
           delbss_params->sessionId, delbss_params->bssIdx,
           MAC_ADDR_ARRAY(session_entry->bssId));

    /* we need to defer the message until we get the response back from HAL. */
    SET_LIM_PROCESS_DEFD_MESGS(mac, false);

    msg.type = WDA_DELETE_BSS_REQ;
    msg.reserved = 0;
    msg.bodyptr = delbss_params;
    msg.bodyval = 0;

    MTRACE(macTraceMsgTx(mac, session_entry->peSessionId, msg.type));

    if(eSIR_SUCCESS != (ret_code = wdaPostCtrlMsg(mac, &msg)))
    {
        SET_LIM_PROCESS_DEFD_MESGS(mac, true);
        limLog(mac, LOGE,
               FL("Posting DELETE_BSS_REQ to HAL failed, reason=%X"), ret_code);
        vos_mem_free(delbss_params);
    }

    return ret_code;
}

/*
 * lim_add_bss_mbb() -performs add bss of new roamable AP
 * @mac: MAC context
 * @sta_ds: station entry
 * @bss_idx:BSS index
 * @session_entry: session entry
 *
 * This function performs add bss of new roamable AP
 */
void lim_add_bss_mbb(tpAniSirGlobal mac, tpDphHashNode sta_ds,
    tpPESession session_entry)
{
    tSirMsgQ msg;
    tSirRetStatus ret_code;

    /* we need to defer the message until we get the response back from HAL. */
    SET_LIM_PROCESS_DEFD_MESGS(mac, false);

    msg.type = SIR_HAL_ADD_BSS_REQ;
    msg.reserved = 0;
    msg.bodyptr = mac->ft.ftPEContext.pAddBssReq;
    msg.bodyval = 0;

    limLog(mac, LOG1, FL("Sending SIR_HAL_ADD_BSS_REQ"));

    session_entry->limMlmState = eLIM_MLM_WT_ADD_BSS_RSP_REASSOC_STATE;

    MTRACE(macTrace(mac, TRACE_CODE_MLM_STATE,
          session_entry->peSessionId, eLIM_MLM_WT_ADD_BSS_RSP_REASSOC_STATE));

    MTRACE(macTraceMsgTx(mac, session_entry->peSessionId, msg.type));

    ret_code = wdaPostCtrlMsg(mac, &msg);
    if( eSIR_SUCCESS != ret_code) {
        vos_mem_free(mac->ft.ftPEContext.pAddBssReq);
        limLog(mac, LOGE,
               FL("Posting ADD_BSS_REQ to HAL failed, reason=%X"), ret_code);
    }
    /* Dont need this anymore */
    mac->ft.ftPEContext.pAddBssReq = NULL;
}

tSirRetStatus lim_add_sta_mbb(tpAniSirGlobal mac, tANI_U16 assoc_id,
    tpPESession session_entry)
{
    tpAddStaParams add_sta_params = NULL;
    tSirMsgQ msg;
    tSirRetStatus retCode = eSIR_SUCCESS;

    add_sta_params = mac->ft.ftPEContext.pAddStaReq;
    add_sta_params->assocId = assoc_id;

    msg.type = SIR_HAL_ADD_STA_REQ;
    msg.reserved = 0;
    msg.bodyptr = add_sta_params;
    msg.bodyval = 0;

    limLog(mac, LOG1,
           FL("Sending SIR_HAL_ADD_STA_REQ. aid %d)"),
           add_sta_params->assocId);

    session_entry->limPrevMlmState = session_entry->limMlmState;
    session_entry->limMlmState = eLIM_MLM_WT_ADD_STA_RSP_STATE;

    MTRACE(macTrace(mac, TRACE_CODE_MLM_STATE,
          session_entry->peSessionId, eLIM_MLM_WT_ADD_STA_RSP_STATE));

    MTRACE(macTraceMsgTx(mac, session_entry->peSessionId, msg.type));

    if(eSIR_SUCCESS != (retCode = wdaPostCtrlMsg( mac, &msg))) {
        limLog(mac, LOGE,
               FL("Posting ADD_STA_REQ to HAL failed, reason=%X"), retCode);
        vos_mem_free(add_sta_params);
    }

    mac->ft.ftPEContext.pAddStaReq = NULL;
    return retCode;
}

/*
 * lim_handle_reassoc_mbb_success() -handles reassoc success
 * @mac: MAC context
 * @session_entry: session entry
 * @assoc_rsp: pointer to assoc response
 * @sta_ds : station entry
 *
 * This function handles reassoc success
 */
void lim_handle_reassoc_mbb_success(tpAniSirGlobal mac,
     tpPESession session_entry, tpSirAssocRsp  assoc_rsp, tpDphHashNode sta_ds)
{
    tpPESession session_entry_con_ap;
    tANI_U8 session_id_connected_ap;
    tpDphHashNode sta_ds_connected_ap;
    tANI_U16 aid;
    tSirRetStatus ret_code;

    limUpdateAssocStaDatas(mac, sta_ds, assoc_rsp, session_entry);

    /* Store assigned AID for TIM processing */
    session_entry->limAID = assoc_rsp->aid & 0x3FFF;

    /* De register STA for currently connected AP */
    mac->sme.roaming_mbb_callback(mac, mac->ft.ftSmeContext.smeSessionId,
                            NULL, NULL, SIR_ROAMING_DEREGISTER_STA);

    mac->sme.roaming_mbb_callback(mac, mac->ft.ftSmeContext.smeSessionId,
                            NULL, NULL, SIR_STOP_ROAM_OFFLOAD_SCAN);

    if((session_entry_con_ap = peFindSessionByBssid(mac,
          mac->ft.ftPEContext.pFTPreAuthReq->currbssId,
          &session_id_connected_ap))== NULL) {
        limLog(mac, LOGE,
               FL("session does not exist for given BSSID" MAC_ADDRESS_STR),
               MAC_ADDR_ARRAY(mac->ft.ftPEContext.pFTPreAuthReq->currbssId));
        goto end;
    }

    session_entry->smeSessionId = session_entry_con_ap->smeSessionId;

    sta_ds_connected_ap = dphLookupHashEntry(mac,
                               mac->ft.ftPEContext.pFTPreAuthReq->currbssId,
                               &aid,
                               &session_entry_con_ap->dph.dphHashTable);
    if (sta_ds_connected_ap == NULL) {
        limLog(mac, LOGE,
               FL("sta_ds NULL for given BSSID" MAC_ADDRESS_STR),
               MAC_ADDR_ARRAY(mac->ft.ftPEContext.pFTPreAuthReq->currbssId));
        goto end;
    }

    /*
     * Change Mlm state of new AP to Del sta rsp state so that
     * duplicate reassoc response will be dropped.
     */
    session_entry->limMlmState = eLIM_MLM_WT_DEL_STA_RSP_STATE;
    MTRACE(macTrace(mac, TRACE_CODE_MLM_STATE,
              session_entry->peSessionId, eLIM_MLM_WT_DEL_STA_RSP_STATE));

    /* Delete sta for currently connected AP */
    ret_code = lim_del_sta_mbb(mac, sta_ds_connected_ap,
                    false, session_entry_con_ap);
    if (ret_code == eSIR_SUCCESS)
        return;

end:
    /* Connected AP lim cleanup.*/
    lim_cleanup_connected_ap(mac, sta_ds_connected_ap, session_entry_con_ap);
    /*
     * eHAL_STATUS_INVALID_PARAMETER is used
     * so that full cleanup is triggered.
     */
    lim_reassoc_fail_cleanup(mac, eHAL_STATUS_INVALID_PARAMETER,
                                (tANI_U32 *)session_entry);
}


/*
 * lim_process_preauth_mbb_result() -process pre auth result
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 *
 * This function invokes resume callback
 */
static inline void lim_process_preauth_mbb_result(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data)
{
    tpPESession session_entry, ft_session_entry;
    tpDphHashNode sta_ds;
    tAddBssParams *add_bss_params;
    tSirSmeJoinReq *reassoc_req;
    tLimMlmReassocReq *mlm_reassoc_req;
    tANI_U16 caps;
    tANI_U16 nSize;
    tpSirSmeJoinReq pReassocReq = NULL;

    if (!mac->ft.ftPEContext.pFTPreAuthReq) {
        limLog(mac, LOG1, "Pre-Auth request is NULL!");
        goto end;
    }

    session_entry = (tpPESession)data;

    /* Post the FT Pre Auth Response to SME in case of failure*/
    if (mac->ft.ftPEContext.ftPreAuthStatus == eSIR_FAILURE)
        goto end;

    /* Flow for preauth success */
    limFTSetupAuthSession(mac, session_entry);

    /*
     * Prepare reassoc request. Memory allocated for tSirSmeJoinReq
     *reassoc_req in csr_fill_reassoc_req. Free that memory here.
     */
    mac->sme.roaming_mbb_callback(mac, mac->ft.ftSmeContext.smeSessionId,
                            mac->ft.ftPEContext.pFTPreAuthReq->pbssDescription,
                            &reassoc_req, SIR_PREPARE_REASSOC_REQ);
    if (reassoc_req  == NULL) {
        limLog(mac, LOGE,
               FL("reassoc req is NULL"));
        goto end;
    }

    nSize = __limGetSmeJoinReqSizeForAlloc((tANI_U8 *) reassoc_req);
    pReassocReq = vos_mem_malloc(nSize);
    if ( NULL == pReassocReq )
    {
        limLog(mac, LOGE,
               FL("call to AllocateMemory failed for pReassocReq"));
        goto end;
    }
    vos_mem_set((void *) pReassocReq, nSize, 0);
    if ((limJoinReqSerDes(mac, (tpSirSmeJoinReq) pReassocReq,
                          (tANI_U8 *) reassoc_req) == eSIR_FAILURE) ||
        (!limIsSmeJoinReqValid(mac,
                               (tpSirSmeJoinReq) pReassocReq)))
    {
        limLog(mac, LOGE,
               FL("received SME_REASSOC_REQ with invalid data"));
        goto end;
    }

    ft_session_entry = mac->ft.ftPEContext.pftSessionEntry;

    ft_session_entry->pLimReAssocReq = pReassocReq;
    vos_mem_free(reassoc_req);

    add_bss_params = mac->ft.ftPEContext.pAddBssReq;

    mlm_reassoc_req = vos_mem_malloc(sizeof(tLimMlmReassocReq));
    if (NULL == mlm_reassoc_req) {
        limLog(mac, LOGE,
               FL("call to AllocateMemory failed for mlmReassocReq"));
        goto end;
    }

    vos_mem_copy(mlm_reassoc_req->peerMacAddr,
                 ft_session_entry->limReAssocbssId,
                 sizeof(tSirMacAddr));
    mlm_reassoc_req->reassocFailureTimeout = PREAUTH_REASSOC_TIMEOUT;

    if (cfgGetCapabilityInfo(mac, &caps, ft_session_entry) != eSIR_SUCCESS) {
        limLog(mac, LOGE, FL("could not retrieve Capabilities value"));
        vos_mem_free(mlm_reassoc_req);
        goto end;
    }

    lim_update_caps_info_for_bss(mac, &caps,
                        reassoc_req->bssDescription.capabilityInfo);

    limLog(mac, LOG1, FL("Capabilities info Reassoc: 0x%X"), caps);

    mlm_reassoc_req->capabilityInfo = caps;
    mlm_reassoc_req->sessionId = ft_session_entry->peSessionId;
    mlm_reassoc_req->listenInterval = WNI_CFG_LISTEN_INTERVAL_STADEF;

    if ((sta_ds = dphAddHashEntry(mac, add_bss_params->bssId,
                  DPH_STA_HASH_INDEX_PEER,
                  &ft_session_entry->dph.dphHashTable)) == NULL) {
        limLog(mac, LOGE, FL("could not add hash entry at DPH"));
        limPrintMacAddr(mac, add_bss_params->bssId, LOGE);
        vos_mem_free(mlm_reassoc_req);
        goto end;
    }

    /* Start timer here to handle reassoc timeout */
    mac->lim.limTimers.glim_reassoc_mbb_rsp_timer.sessionId =
                                                ft_session_entry->peSessionId;

    MTRACE(macTrace(mac, TRACE_CODE_TIMER_ACTIVATE,
             session_entry->peSessionId, eLIM_REASSOC_MBB_RSP_TIMER));

    if(TX_SUCCESS !=
          tx_timer_activate(&mac->lim.limTimers.glim_reassoc_mbb_rsp_timer)) {
       limLog(mac, LOGE, FL("Reassoc MBB Rsp Timer Start Failed"));

       if (ft_session_entry->pLimReAssocReq) {
           vos_mem_free(ft_session_entry->pLimReAssocReq);
           ft_session_entry->pLimReAssocReq = NULL;
       }

       vos_mem_free(mlm_reassoc_req);
       goto end;
    }

    limLog(mac, LOG1, FL("enabling caching"));
    WLANTL_EnablePreAssocCaching();

    /* To do: Add changes for reassoc fail timer */
    limSendReassocReqWithFTIEsMgmtFrame(mac,
                     mlm_reassoc_req, ft_session_entry);

    ft_session_entry->limMlmState = eLIM_MLM_WT_REASSOC_RSP_STATE;

    limLog(mac, LOG1,  FL("Set the mlm state to %d session=%d"),
           ft_session_entry->limMlmState, ft_session_entry->peSessionId);
    return;

end:
    lim_handle_reassoc_mbb_fail(mac, ft_session_entry);
}

/*
 * lim_perform_post_preauth_mbb_channel_change() -invokes resume callback
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 * @session_entry: session entry
 *
 * This function invokes resume callback after successful reception of
 * pre auth
 */
static inline
void lim_perform_post_preauth_mbb_channel_change(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data, tpPESession session_entry)
{
    peSetResumeChannel(mac, 0, 0);
    limResumeLink(mac, lim_process_preauth_mbb_result,
                                (tANI_U32 *)session_entry);
}

/*
 * lim_handle_pre_auth_mbb_rsp() -handles preauth response
 * @mac: MAC context
 * @status: status of message
 * @session_entry: session entry
 *
 * This function process preauth response
 */
void lim_handle_pre_auth_mbb_rsp(tpAniSirGlobal mac,
     tSirRetStatus status, tpPESession session_entry)
{
    tpPESession ft_session_entry;
    tANI_U8 session_id;
    tpSirBssDescription  bss_description;

    mac->ft.ftPEContext.ftPreAuthStatus = status;

    mac->ft.ftPEContext.saved_auth_rsp_length = 0;

    limLog(mac, LOG1, FL("preauth status %d"),
                         mac->ft.ftPEContext.ftPreAuthStatus);

    /* Create FT session for the re-association at this point */
    if (mac->ft.ftPEContext.ftPreAuthStatus == eSIR_SUCCESS) {
        bss_description = mac->ft.ftPEContext.pFTPreAuthReq->pbssDescription;
        if((ft_session_entry = peCreateSession(mac, bss_description->bssId,
                                  &session_id, mac->lim.maxStation)) == NULL) {
            limLog(mac, LOGE,
                   FL("session can not be created for pre-auth AP"));
            mac->ft.ftPEContext.ftPreAuthStatus = eSIR_FAILURE;
            goto out;
        }
        ft_session_entry->peSessionId = session_id;
        sirCopyMacAddr(ft_session_entry->selfMacAddr,
                                 session_entry->selfMacAddr);
        sirCopyMacAddr(ft_session_entry->limReAssocbssId,
                                     bss_description->bssId);
        ft_session_entry->bssType = session_entry->bssType;

        if (ft_session_entry->bssType == eSIR_INFRASTRUCTURE_MODE)
            ft_session_entry->limSystemRole = eLIM_STA_ROLE;

        ft_session_entry->limSmeState = eLIM_SME_WT_REASSOC_STATE;
        mac->ft.ftPEContext.pftSessionEntry = ft_session_entry;
        limLog(mac, LOG1,"%s:created session (%pK) with id = %d",
               __func__, ft_session_entry, ft_session_entry->peSessionId);

        /* Update the ReAssoc BSSID of the current session */
        sirCopyMacAddr(session_entry->limReAssocbssId, bss_description->bssId);
        limPrintMacAddr(mac, session_entry->limReAssocbssId, LOG1);

        /* Prepare session for roamable AP */
        lim_process_preauth_mbb_result(mac,
               mac->ft.ftPEContext.ftPreAuthStatus, (tANI_U32 *)session_entry);
        return;
    }else {
        lim_handle_preauth_mbb_fail(mac, session_entry);
        return;
    }

out:
    /* This sequence needs to be executed in case of failure*/
    if (session_entry->currentOperChannel !=
        mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum) {
        limChangeChannelWithCallback(mac, session_entry->currentOperChannel,
              lim_perform_post_preauth_mbb_channel_change, NULL, session_entry);
     } else {
        /* Link needs to be resumed as link was suspended for same channel */
        peSetResumeChannel(mac, 0, 0);
        limResumeLink(mac, lim_process_preauth_mbb_result,
                                           (tANI_U32 *)session_entry);
     }
}

/**
 * lim_process_preauth_mbb_rsp_timeout() -Process preauth response timeout
 * @mac: MAC context
 *
 * This function is called if preauth response is not received from the AP
 * within timeout
 */
void lim_process_preauth_mbb_rsp_timeout(tpAniSirGlobal mac)
{
    tpPESession session_entry;

    /*
     * Pre auth is failed. Need to resume link and get back on
     * to home channel.
     */
    limLog(mac, LOG1, FL("Pre-Auth MBB Time Out!!!!"));

    if((session_entry = peFindSessionBySessionId(mac,
        mac->lim.limTimers.glim_pre_auth_mbb_rsp_timer.sessionId))== NULL) {
        limLog(mac, LOGE, FL("session does not exist for given session id"));
        return;
    }

    /*
     * To handle the race condition where we recieve preauth rsp after
     * timer has expired.
     */
    if (mac->ft.ftPEContext.pFTPreAuthReq == NULL) {
        limLog(mac, LOGE, FL("Auth Rsp might already be posted to SME"
               "and cleanup done! sessionId:%d"),
                mac->lim.limTimers.glim_pre_auth_mbb_rsp_timer.sessionId);
        return;
    }

    if (eANI_BOOLEAN_TRUE ==
         mac->ft.ftPEContext.pFTPreAuthReq->bPreAuthRspProcessed) {
         limLog(mac, LOGE,
                FL("Auth rsp already posted to SME session %pK"), session_entry);
         return;
    } else {
    /*
     * Here we are sending preauth rsp with failure state
     * and which is forwarded to SME. Now, if we receive an preauth
     * resp from AP with success it would create a FT pesession, but
     * will be dropped in SME leaving behind the pesession.
     * Mark Preauth rsp processed so that any rsp from AP is dropped in
     * limProcessAuthFrameNoSession.
     */
     limLog(mac,LOG1,
            FL("Auth rsp not yet posted to SME session %pK)"), session_entry);
            mac->ft.ftPEContext.pFTPreAuthReq->bPreAuthRspProcessed =
           eANI_BOOLEAN_TRUE;
     }
     /*
      * Ok, so attempted a Pre-Auth and failed. If we are off channel. We need
      * to get back.
      */
     lim_handle_pre_auth_mbb_rsp(mac, eSIR_FAILURE, session_entry);
 }

/**
 * lim_process_reassoc_mbb_rsp_timeout() -Process reassoc response timeout
 * @mac: MAC context
 *
 * This function is called if preauth response is not received from the
 * AP within timeout
 */
void lim_process_reassoc_mbb_rsp_timeout(tpAniSirGlobal mac)
{
    tpPESession session_entry, ft_session_entry;
    tANI_U8 session_id;

    if((ft_session_entry = peFindSessionBySessionId(mac,
        mac->lim.limTimers.glim_reassoc_mbb_rsp_timer.sessionId))== NULL) {
        limLog(mac, LOGE,
               FL("ft session does not exist for given session id %d"),
               mac->lim.limTimers.glim_reassoc_mbb_rsp_timer.sessionId);
        return;
    }

    limLog(mac, LOG1, FL("Reassoc timeout happened in state %d"),
                         ft_session_entry->limMlmState);

    if((session_entry = peFindSessionByBssid(mac,
          mac->ft.ftPEContext.pFTPreAuthReq->currbssId, &session_id))== NULL) {
        limLog(mac, LOGE,
               FL("session does not exist for given BSSID" MAC_ADDRESS_STR),
               MAC_ADDR_ARRAY(mac->ft.ftPEContext.pFTPreAuthReq->currbssId));
        return;
    }

    lim_handle_reassoc_mbb_fail(mac, ft_session_entry);

}


/**
 * lim_perform_pre_auth_reassoc() -Sends preauth request
 * @mac: MAC context
 * @status: status of message
 * @data: gives information of session
 * @session_entry: session entry
 *
 * This function process preauth request received from CSR
 */
static inline
void lim_perform_pre_auth_reassoc(tpAniSirGlobal mac, eHalStatus status,
     tANI_U32 *data, tpPESession session_entry)
{
    tSirMacAuthFrameBody authFrame;

    if (status != eHAL_STATUS_SUCCESS) {
        limLog(mac, LOGE,
               FL("Change channel not successful for pre-auth"));
        goto preauth_fail;
    }

    limLog(mac, LOGE,
           FL("session id %d"), session_entry->peSessionId);

    mac->ft.ftPEContext.psavedsessionEntry = session_entry;

    authFrame.authAlgoNumber = eSIR_OPEN_SYSTEM;
    authFrame.authTransactionSeqNumber = SIR_MAC_AUTH_FRAME_1;
    authFrame.authStatusCode = 0;

    /* Start timer here to come back to operating channel. */
    mac->lim.limTimers.glim_pre_auth_mbb_rsp_timer.sessionId =
                                                session_entry->peSessionId;

    limSendAuthMgmtFrame(mac, &authFrame,
         mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId,
         LIM_NO_WEP_IN_FC, session_entry, eSIR_FALSE);

    MTRACE(macTrace(mac, TRACE_CODE_TIMER_ACTIVATE,
             session_entry->peSessionId, eLIM_PREAUTH_MBB_RSP_TIMER));

    if(TX_SUCCESS !=
          tx_timer_activate(&mac->lim.limTimers.glim_pre_auth_mbb_rsp_timer)) {
       limLog(mac, LOGE, FL("Pre Auth MBB Rsp Timer Start Failed"));

       mac->ft.ftPEContext.psavedsessionEntry = NULL;
       goto preauth_fail;
    }

    return;

preauth_fail:
     lim_handle_pre_auth_mbb_rsp(mac, eSIR_FAILURE, session_entry);
     return;
}

/**
 * pre_auth_mbb_suspend_link_handler() -Handler for suspend link
 * @mac: MAC context
 * @status: status of message
 * @data: gives information of session
 *
 * This function process preauth request received from CSR
 */
static inline
void pre_auth_mbb_suspend_link_handler(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data)
{
    tpPESession session_entry;

    if (status != eHAL_STATUS_SUCCESS) {
        limLog(mac, LOGE, FL("Link suspend failed"));
        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE,
                        (tpPESession)data, SIR_MBB_CONNECTED);
        return;
    }

    session_entry = (tpPESession)data;

    if (session_entry->currentOperChannel !=
                mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum) {
        limChangeChannelWithCallback(mac,
                mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum,
                lim_perform_pre_auth_reassoc, NULL, session_entry);
        return;
    } else {
        lim_perform_pre_auth_reassoc(mac, eHAL_STATUS_SUCCESS,
                                  NULL, session_entry);
        return;
    }
}

/**
 * lim_process_pre_auth_reassoc_req() -Process preauth request
 * @hal: HAL context
 * @msg: message
 *
 * This function process preauth request received from CSR
 */
void lim_process_pre_auth_reassoc_req(tpAniSirGlobal mac, tpSirMsgQ msg)
{
    tpPESession session_entry;
    tANI_U8 session_id;

    limFTInit(mac);

    /* Can set it only after sending auth */
    mac->ft.ftPEContext.ftPreAuthStatus = eSIR_FAILURE;

    /* We need information from the Pre-Auth Req. Lets save that */
    mac->ft.ftPEContext.pFTPreAuthReq = (tpSirFTPreAuthReq)msg->bodyptr;
    if (!mac->ft.ftPEContext.pFTPreAuthReq) {
        limLog(mac, LOGE,
               FL("pFTPreAuthReq is NULL"));
        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                                  SIR_MBB_CONNECTED);
        return;
    }

    /* Get the current session entry */
    session_entry = peFindSessionByBssid(mac,
                    mac->ft.ftPEContext.pFTPreAuthReq->currbssId, &session_id);
    if (session_entry == NULL) {
        limLog(mac, LOGE,
               FL("Unable to find session for the following bssid"));
        limPrintMacAddr(mac,
                        mac->ft.ftPEContext.pFTPreAuthReq->currbssId, LOGE);

        /* Post the pre auth response to SME */
        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                                SIR_MBB_CONNECTED);
        return;
    }

    limLog(mac, LOG1,
           FL("set link with eSIR_LINK_PRE_AUTH_REASSOC_STATE"));

    if (limSetLinkState(mac, eSIR_LINK_PRE_AUTH_REASSOC_STATE,
                        session_entry->bssId, session_entry->selfMacAddr,
                        NULL, NULL) != eSIR_SUCCESS) {
        limLog(mac, LOGE,
               FL("set link failed for eSIR_LINK_PRE_AUTH_REASSOC_STATE"));
        lim_post_pre_auth_reassoc_rsp(mac, eSIR_FAILURE, NULL,
                                                   SIR_MBB_CONNECTED);
        return;
    }

    /*
     * Suspend link for same channel or different channel so that STA
     * can be in power save for connected AP.
     */
    limLog(mac, LOG1,
           FL("pre-auth on channel %d (session %pK) currentOperChannel %d"),
           mac->ft.ftPEContext.pFTPreAuthReq->preAuthchannelNum,
           session_entry, session_entry->currentOperChannel);
    limSuspendLink(mac, eSIR_CHECK_ROAMING_SCAN,
                   pre_auth_mbb_suspend_link_handler,
                  (tANI_U32 *)session_entry);
}


/**
 * lim_process_sta_mlm_del_sta_rsp_mbb() -Process del sta response
 * @mac: MAC context
 * @lim_msg: lim message
 * @session_entry: session entry
 *
 * This function process del sta response
 */
void lim_process_sta_mlm_del_sta_rsp_mbb(tpAniSirGlobal mac,
     tpSirMsgQ lim_msg, tpPESession session_entry)
{
    tpDeleteStaParams del_sta_params = (tpDeleteStaParams)lim_msg->bodyptr;
    tpDphHashNode sta_ds = NULL;
    tANI_U8 session_id;
    tpPESession ft_session_entry;

    if(NULL == del_sta_params) {
       limLog(mac, LOGE, FL("Encountered NULL Pointer"));
       goto end;
    }

    limLog(mac, LOG1, FL("Del STA RSP received. Status:%d AssocID:%d"),
           del_sta_params->status, del_sta_params->assocId);

    if (eHAL_STATUS_SUCCESS != del_sta_params->status) {
        limLog(mac, LOGE, FL("Del STA failed! Status:%d, still proceeding"
               "with Del BSS"), del_sta_params->status);
    }

    sta_ds = dphGetHashEntry(mac, DPH_STA_HASH_INDEX_PEER,
                             &session_entry->dph.dphHashTable);

    if (sta_ds == NULL) {
        limLog( mac, LOGE, FL("DPH Entry for STA %X missing"),
                del_sta_params->assocId);
        goto end;
    }

    if (eLIM_MLM_WT_DEL_STA_RSP_STATE != session_entry->limMlmState) {
        limLog(mac, LOGE,
               FL( "Received unexpected WDA_DELETE_STA_RSP in state %s" ),
               limMlmStateStr(session_entry->limMlmState));
        goto end;
    }

    limLog( mac, LOG1, FL("STA AssocID %d MAC "), sta_ds->assocId);
    limPrintMacAddr(mac, sta_ds->staAddr, LOG1);

    /*
     * we must complete all cleanup related to del sta
     * before calling del bss.
     */
    if (NULL != lim_msg->bodyptr) {
        vos_mem_free(del_sta_params);
        lim_msg->bodyptr = NULL;
    }

    /* Proceed to do del bss even if del sta resulted in failure */
    lim_del_bss_mbb(mac, sta_ds, 0, session_entry);
    return;

end:
    if(NULL != lim_msg->bodyptr) {
        vos_mem_free(del_sta_params);
        lim_msg->bodyptr = NULL;
    }

    /* Connected AP lim cleanup.*/
    lim_cleanup_connected_ap(mac, sta_ds, session_entry);

    ft_session_entry = peFindSessionByBssid(mac,
                         mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId,
                         &session_id);
    if (ft_session_entry == NULL) {
        limLog(mac, LOGE,
               FL("Unable to find session for the following bssid"));
        limPrintMacAddr(mac,
                     mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId, LOGE);
        return;
    }

    /*
     * eHAL_STATUS_INVALID_PARAMETER is used
     * so that full cleanup is triggered.
     */
    lim_reassoc_fail_cleanup(mac, eHAL_STATUS_INVALID_PARAMETER,
                                (tANI_U32 *)ft_session_entry);
}

/**
 * lim_cleanup_rx_path_mbb() -cleans up tspec related info
 * @mac: MAC context
 * @sta_ds: station entry
 * @session_entry: session entry of connected AP
 *
 * This function cleans up tspec related info
 */
void lim_cleanup_rx_path_mbb(tpAniSirGlobal mac,
    tpDphHashNode sta_ds,tpPESession session_entry)
{
    limLog(mac, LOG1, FL("AID %d limSmeState %d, mlmState %d"),
           sta_ds->assocId, session_entry->limSmeState,
           sta_ds->mlmStaContext.mlmState);

    session_entry->isCiscoVendorAP = FALSE;

    if (mac->lim.gLimAddtsSent)
        tx_timer_deactivate(&mac->lim.limTimers.gLimAddtsRspTimer);

    /* delete all tspecs associated with this sta. */
    limAdmitControlDeleteSta(mac, sta_ds->assocId);
}


/**
 * lim_cleanup_connected_ap() -cleans up connected AP lim info
 * @mac: MAC context
 * @sta_ds: station entry
 * @session_entry: session entry of connected AP
 *
 * This function cleans up connected AP lim info
 */
void lim_cleanup_connected_ap(tpAniSirGlobal mac, tpDphHashNode sta_ds,
     tpPESession session_entry)
{
    lim_cleanup_rx_path_mbb(mac, sta_ds, session_entry);
    limDeleteDphHashEntry(mac, sta_ds->staAddr,
                          sta_ds->assocId, session_entry);

    /**
     * Make STA hash entry invalid at eCPU so that DPH
     * does not process any more data packets and
     * releases those BDs
     */
    sta_ds->valid = 0;

    peDeleteSession(mac, session_entry);
}

/**
 * lim_process_sta_mlm_del_bss_rsp_mbb() -Process del bss response of
 * connected AP
 * @mac: MAC context
 * @lim_msg: lim message
 * @session_entry: session entry of connected AP
 *
 * This function process del sta response
 */
void lim_process_sta_mlm_del_bss_rsp_mbb(tpAniSirGlobal mac,
     tpSirMsgQ lim_msg, tpPESession session_entry)
{
    tpDeleteBssParams delbss_params = (tpDeleteBssParams)lim_msg->bodyptr;
    tpDphHashNode sta_ds = dphGetHashEntry(mac, DPH_STA_HASH_INDEX_PEER,
                                            &session_entry->dph.dphHashTable);
    tpPESession ft_session_entry;
    tANI_U8 session_id;

    if (NULL == delbss_params) {
        limLog(mac, LOGE, FL( "Invalid body pointer in message"));
        goto end;
    }

    ft_session_entry = peFindSessionByBssid(mac,
                         mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId,
                         &session_id);
    if (ft_session_entry == NULL) {
        limLog(mac, LOGE,
               FL("Unable to find session for the following bssid"));
        limPrintMacAddr(mac,
                     mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId, LOGE);
        goto end;
    }

    if(eHAL_STATUS_SUCCESS == delbss_params->status) {
       limLog(mac, LOG1,
              FL( "STA received the DEL_BSS_RSP for BSSID: %X."),
              delbss_params->bssIdx);

       if(sta_ds == NULL) {
          limLog(mac, LOGE, FL("DPH Entry for STA missing"));
          goto end;
       }
       if(eLIM_MLM_WT_DEL_BSS_RSP_STATE != session_entry->limMlmState) {
          limLog(mac, LOGE,
                 FL("Received unexpected WDA_DEL_BSS_RSP in state %d"),
                    session_entry->limMlmState);
          goto end;
       }
       limLog(mac, LOG1, FL("STA AssocID %d MAC "), sta_ds->assocId );
       limPrintMacAddr(mac, sta_ds->staAddr, LOG1);

       lim_cleanup_connected_ap(mac, sta_ds, session_entry);

       lim_add_bss_mbb(mac, sta_ds, ft_session_entry);

       if(NULL != lim_msg->bodyptr) {
          vos_mem_free(delbss_params);
          lim_msg->bodyptr = NULL;
       }
       return;
    } else {
       /*
        * If del bss response is failure, cleanup sessions of both currently
        * connected AP and roamable AP as add bss can not be send
        * without successful delbss.
        */
       limLog(mac, LOGE,
              FL("DEL BSS failed! Status:%d"), delbss_params->status);

       if(NULL != lim_msg->bodyptr) {
          vos_mem_free(delbss_params);
          lim_msg->bodyptr = NULL;
       }

       /* Connected AP lim cleanup.*/
       lim_cleanup_connected_ap(mac, sta_ds, session_entry);

       /* Newly created session cleanup */
       lim_reassoc_fail_cleanup(mac, eHAL_STATUS_MBB_DEL_BSS_FAIL,
                                (tANI_U32 *)ft_session_entry);
       return;
    }

end:
    if(NULL != lim_msg->bodyptr) {
       vos_mem_free(delbss_params);
       lim_msg->bodyptr = NULL;
    }

    lim_cleanup_connected_ap(mac, sta_ds, session_entry);

    ft_session_entry = peFindSessionByBssid(mac,
                         mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId,
                         &session_id);
    if (ft_session_entry == NULL) {
        limLog(mac, LOGE,
               FL("Unable to find session for the following bssid"));
        limPrintMacAddr(mac,
                     mac->ft.ftPEContext.pFTPreAuthReq->preAuthbssId, LOGE);
        return;
    }

    /*
     * eHAL_STATUS_INVALID_PARAMETER is used
     * so that full cleanup is triggered.
     */
    lim_reassoc_fail_cleanup(mac, eHAL_STATUS_INVALID_PARAMETER,
                                (tANI_U32 *)ft_session_entry);
}


void lim_fill_add_sta_params_mbb(tpAniSirGlobal mac,
     tpPESession session_entry, tpAddStaParams add_sta_params)
{
    tANI_U32 listen_interval = WNI_CFG_LISTEN_INTERVAL_STADEF;
    tANI_U32 self_sta_dot11_mode = 0;

    vos_mem_copy(add_sta_params->staMac, session_entry->selfMacAddr,
                 sizeof(tSirMacAddr));
    vos_mem_copy(add_sta_params->bssId, session_entry->bssId,
                 sizeof(tSirMacAddr));

    add_sta_params->staType = STA_ENTRY_SELF;
    add_sta_params->status = eHAL_STATUS_SUCCESS;
    add_sta_params->respReqd = 1;

    add_sta_params->sessionId = session_entry->peSessionId;
    add_sta_params->staIdx = HAL_STA_INVALID_IDX;
    add_sta_params->updateSta = FALSE;

    add_sta_params->shortPreambleSupported =
                 session_entry->beaconParams.fShortPreamble;

#ifdef WLAN_FEATURE_11AC
    limPopulatePeerRateSet(mac, &add_sta_params->supportedRates, NULL,
                           false,session_entry, NULL);
#else
    limPopulatePeerRateSet(mac, &add_sta_params->supportedRates, NULL,
                           false,session_entry);
#endif

    if(session_entry->htCapability) {
        add_sta_params->htCapable = session_entry->htCapability;
#ifdef WLAN_FEATURE_11AC
        add_sta_params->vhtCapable = session_entry->vhtCapability;
        add_sta_params->vhtTxChannelWidthSet = session_entry->vhtTxChannelWidthSet;
#endif

#ifdef DISABLE_GF_FOR_INTEROP
        if((session_entry->pLimJoinReq != NULL) &&
               (!session_entry->pLimJoinReq->bssDescription.aniIndicator)) {
            limLog(mac, LOGE,
                   FL("Turning off Greenfield, when adding self entry"));
            add_sta_params->greenFieldCapable =
                          WNI_CFG_GREENFIELD_CAPABILITY_DISABLE;
        }
        else
#endif
        add_sta_params->greenFieldCapable =
                       limGetHTCapability( mac, eHT_GREENFIELD, session_entry);

        if (session_entry->limRFBand == SIR_BAND_2_4_GHZ)
            add_sta_params->txChannelWidthSet =
                     mac->roam.configParam.channelBondingMode24GHz;
        else
            add_sta_params->txChannelWidthSet =
                     mac->roam.configParam.channelBondingMode5GHz;

        add_sta_params->mimoPS =
              limGetHTCapability(mac, eHT_MIMO_POWER_SAVE, session_entry);
        add_sta_params->rifsMode =
              limGetHTCapability(mac, eHT_RIFS_MODE, session_entry);
        add_sta_params->lsigTxopProtection =
              limGetHTCapability(mac, eHT_LSIG_TXOP_PROTECTION, session_entry);
        add_sta_params->delBASupport =
              limGetHTCapability(mac, eHT_DELAYED_BA, session_entry);
        add_sta_params->maxAmpduDensity =
              limGetHTCapability(mac, eHT_MPDU_DENSITY, session_entry);
        add_sta_params->maxAmpduSize =
              limGetHTCapability(mac, eHT_MAX_RX_AMPDU_FACTOR, session_entry);
        add_sta_params->maxAmsduSize =
              limGetHTCapability(mac, eHT_MAX_AMSDU_LENGTH, session_entry);
        add_sta_params->fDsssCckMode40Mhz =
              limGetHTCapability(mac, eHT_DSSS_CCK_MODE_40MHZ, session_entry);
        add_sta_params->fShortGI20Mhz =
              limGetHTCapability(mac, eHT_SHORT_GI_20MHZ, session_entry);
        add_sta_params->fShortGI40Mhz =
              limGetHTCapability(mac, eHT_SHORT_GI_40MHZ, session_entry);
    }

    if (wlan_cfgGetInt(mac, WNI_CFG_LISTEN_INTERVAL,
                         &listen_interval) != eSIR_SUCCESS)
        limLog(mac, LOGE, FL("Couldn't get LISTEN_INTERVAL"));

    add_sta_params->listenInterval = (tANI_U16)listen_interval;

    wlan_cfgGetInt(mac, WNI_CFG_DOT11_MODE, &self_sta_dot11_mode);
    add_sta_params->supportedRates.opRateMode =
                       limGetStaRateMode((tANI_U8)self_sta_dot11_mode);

    mac->ft.ftPEContext.pAddStaReq = add_sta_params;


}

void lim_process_sta_mlm_add_bss_rsp_mbb(tpAniSirGlobal mac,
    tpSirMsgQ limMsgQ,tpPESession session_entry)
{
    tpAddBssParams add_bss_params = (tpAddBssParams)limMsgQ->bodyptr;
    tpAddStaParams add_sta_params = NULL;
    tpDphHashNode sta_ds = dphGetHashEntry(mac, DPH_STA_HASH_INDEX_PEER,
                                        &session_entry->dph.dphHashTable);

    if(add_bss_params == 0)
        goto end;

    limLog(mac, LOG1, FL("Add BSS RSP received. Status:%d"),
                         add_bss_params->status);

    if(eHAL_STATUS_SUCCESS == add_bss_params->status) {

       if(eLIM_MLM_WT_ADD_BSS_RSP_REASSOC_STATE != session_entry->limMlmState) {
          limLog(mac, LOGE,
                 FL("Received unexpected ADD BSS in state %d"),
                    session_entry->limMlmState);
          goto end;
       }

       session_entry->bssIdx = add_bss_params->bssIdx;

       sta_ds->bssId = add_bss_params->bssIdx;
       sta_ds->staIndex = add_bss_params->staContext.staIdx;
       sta_ds->ucUcastSig = add_bss_params->staContext.ucUcastSig;
       sta_ds->ucBcastSig = add_bss_params->staContext.ucBcastSig;

       rrmCacheMgmtTxPower(mac, add_bss_params->txMgmtPower, session_entry);

       /* Allocate memory for add sta params */
       add_sta_params = vos_mem_malloc(sizeof( tAddStaParams ));
       if (NULL == add_sta_params) {
           limLog(mac, LOGE,
                  FL("Unable to allocate memory during ADD_STA"));
           goto end;
       }
       vos_mem_set(add_sta_params, sizeof(tAddStaParams), 0);

       /* Prepare add sta params */
       lim_fill_add_sta_params_mbb(mac, session_entry, add_sta_params);

       if(0 != limMsgQ->bodyptr) {
          vos_mem_free(add_bss_params);
          add_bss_params = NULL;
          limMsgQ->bodyptr = NULL;
       }

       lim_add_sta_mbb(mac, session_entry->limAID, session_entry);

    } else {
       /*
        * If add bss response is failure, cleanup session of roamable AP
        * as cleanup of connected bss happened during del bss
        */
       limLog(mac, LOGE,
              FL("ADD BSS failed! Status:%d"), add_bss_params->status);

       /* Newly created session cleanup */
       lim_reassoc_fail_cleanup(mac, eHAL_STATUS_MBB_ADD_BSS_FAIL,
                                (tANI_U32 *)session_entry);
    }

end:
    if(0 != limMsgQ->bodyptr) {
        vos_mem_free(add_bss_params);
        add_bss_params = NULL;
        limMsgQ->bodyptr = NULL;
    }
}

/*
 * lim_perform_post_add_sta_rsp() -invokes resume callback to handle add sta response
 * @mac: MAC context
 * @status: status
 * @data: pointer to data
 *
 * This function invokes resume callback
 */
void lim_perform_post_add_sta_rsp(tpAniSirGlobal mac,
     eHalStatus status, tANI_U32 *data)
{
    tpDphHashNode sta_ds;
    /* session entry for newly roamed ap */
    tpPESession session_entry;

    session_entry = (tpPESession)data;

    limLog(mac, LOG1, FL("forwarding cached packets"));
    WLANTL_PreAssocForward(true);

    sta_ds = dphGetHashEntry(mac, DPH_STA_HASH_INDEX_PEER,
                              &session_entry->dph.dphHashTable);
    if(NULL != sta_ds)
       sta_ds->mlmStaContext.mlmState = eLIM_MLM_LINK_ESTABLISHED_STATE;
    else {
       limLog(mac, LOGE,
              FL("Unable to get the DPH Hash Entry for AID - %d"),
              DPH_STA_HASH_INDEX_PEER);
       return;
    }

    session_entry->limMlmState = eLIM_MLM_LINK_ESTABLISHED_STATE;
    MTRACE(macTrace(mac, TRACE_CODE_MLM_STATE,
          session_entry->peSessionId, eLIM_MLM_LINK_ESTABLISHED_STATE));

#ifdef FEATURE_WLAN_TDLS
           /* initialize TDLS peer related data */
           limInitTdlsData(mac, session_entry);
#endif

    if (limSetLinkState(mac, eSIR_LINK_POSTASSOC_STATE, session_entry->bssId,
               session_entry->selfMacAddr, NULL, NULL) != eSIR_SUCCESS) {
        limLog(mac, LOGE, FL("Set link state to POSTASSOC failed"));
        return;
    }

    session_entry->limSmeState = eLIM_SME_LINK_EST_STATE;

    lim_post_pre_auth_reassoc_rsp(mac, eSIR_SUCCESS, session_entry,
                                         SIR_MBB_CONNECTED);

    /* Freeup the cached preauth request */
    if (mac->ft.ftPEContext.pFTPreAuthReq) {
        limLog(mac, LOG1, FL("Freeing pFTPreAuthReq= %pK"),
                             mac->ft.ftPEContext.pFTPreAuthReq);
        if (mac->ft.ftPEContext.pFTPreAuthReq->pbssDescription) {
            vos_mem_free(mac->ft.ftPEContext.pFTPreAuthReq->pbssDescription);
            mac->ft.ftPEContext.pFTPreAuthReq->pbssDescription = NULL;
        }
        vos_mem_free(mac->ft.ftPEContext.pFTPreAuthReq);
        mac->ft.ftPEContext.pFTPreAuthReq = NULL;
    }
}

void lim_process_sta_mlm_add_sta_rsp_mbb(tpAniSirGlobal mac,
    tpSirMsgQ limMsgQ,tpPESession session_entry)
{
    tpAddStaParams add_sta_params = (tpAddStaParams)limMsgQ->bodyptr;

    if(NULL == add_sta_params) {
        limLog(mac, LOGE, FL("Encountered NULL Pointer"));
        return;
    }

    limLog(mac, LOG1, FL("Add STA RSP received. Status:%d"),
                         add_sta_params->status);

    if (eHAL_STATUS_SUCCESS == add_sta_params->status) {
        if ( eLIM_MLM_WT_ADD_STA_RSP_STATE != session_entry->limMlmState) {
            limLog(mac, LOGE,
                   FL("Received unexpected ADD_STA_RSP in state %d"),
                   session_entry->limMlmState);
            goto end;
        }

        /*
         * Storing the self StaIndex(Generated by HAL) in session context,
         * instead of storing it in DPH Hash entry for Self STA.
         * DPH entry for the self STA stores the sta index for the BSS entry
         * to which the STA is associated.
         */
        session_entry->staId = add_sta_params->staIdx;

        limLog(mac, LOG1, FL("resuming to oper ch %d"),
               session_entry->currentOperChannel);

        peSetResumeChannel(mac, session_entry->currentOperChannel, 0);
        limResumeLink(mac, lim_perform_post_add_sta_rsp, (tANI_U32 *)session_entry);
    } else {
       limLog(mac, LOGE, FL( "ADD_STA failed!"));

        /* Newly created session cleanup */
        lim_reassoc_fail_cleanup(mac, eHAL_STATUS_MBB_ADD_BSS_FAIL,
                                (tANI_U32 *)session_entry);

    }
end:
    if( 0 != limMsgQ->bodyptr )
    {
      vos_mem_free(add_sta_params);
      limMsgQ->bodyptr = NULL;
    }
}

eAniBoolean lim_is_mbb_reassoc_in_progress(tpAniSirGlobal mac,
            tpPESession session_entry)
{
    if (session_entry == NULL)
        return eANI_BOOLEAN_FALSE;

    if ((eLIM_STA_ROLE == session_entry->limSystemRole) &&
                        mac->ft.ftSmeContext.is_preauth_lfr_mbb) {
        limLog(mac, LOG1, FL("MBB Reassoc in progress"));
        return eANI_BOOLEAN_TRUE;
    }

    return eANI_BOOLEAN_FALSE;
}
