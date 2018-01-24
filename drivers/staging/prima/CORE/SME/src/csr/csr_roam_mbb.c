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
#include "smeInside.h"
#include "csrInsideApi.h"
#include "smsDebug.h"
#include "macTrace.h"
#include "csrNeighborRoam.h"
#include "csr_roam_mbb.h"
#include "csrInternal.h"
#include "wlan_qct_wda.h"

eHalStatus csr_register_roaming_mbb_callback(tpAniSirGlobal mac);

#define PREAUTH_REASSOC_MBB_TIMER_VALUE    60

#define CSR_NEIGHBOR_ROAM_STATE_TRANSITION(newState)\
{\
    mac->roam.neighborRoamInfo.prevNeighborRoamState = mac->roam.neighborRoamInfo.neighborRoamState;\
    mac->roam.neighborRoamInfo.neighborRoamState = newState;\
    VOS_TRACE (VOS_MODULE_ID_SME, VOS_TRACE_LEVEL_DEBUG, \
               FL("Neighbor Roam Transition from state %s ==> %s"), \
               csrNeighborRoamStateToString (mac->roam.neighborRoamInfo.prevNeighborRoamState), \
               csrNeighborRoamStateToString (newState));\
}

/**
 * csr_roam_issue_preauth_reassoc_req() -Prepares preauth request
 * @hal: HAL context
 * @session_id: session id
 * @bss_description: BSS description
 *
 * This function prepares preauth request and sends request to PE
 *
 * Return: eHAL_STATUS_SUCCESS on success,
 *           : eHAL_STATUS_RESOURCES when resource allocation is failure
 *           : eHAL_STATUS_FAILURE otherwise
 */
eHalStatus csr_roam_issue_preauth_reassoc_req(tHalHandle hal,
                     tANI_U32 session_id, tpSirBssDescription bss_description)
{
    tpAniSirGlobal mac = PMAC_STRUCT(hal);
    tpSirFTPreAuthReq pre_auth_req;
    tANI_U16 auth_req_len = 0;
    tCsrRoamSession *session = CSR_GET_SESSION(mac, session_id);
    tpCsrNeighborRoamControlInfo neighbor_roam_info =
                                          &mac->roam.neighborRoamInfo;
    eHalStatus status;

    auth_req_len = sizeof(tSirFTPreAuthReq);
    pre_auth_req = (tpSirFTPreAuthReq)vos_mem_malloc(auth_req_len);
    if (NULL == pre_auth_req) {
        smsLog(mac, LOGE,
               FL("Memory allocation for Preauth request failed"));
        return eHAL_STATUS_RESOURCES;
    }

    /*
     * If neighborRoamState is eCSR_NEIGHBOR_ROAM_STATE_INIT
     * by the time this API is invoked, disconnect would have happened.
     * So, need to proceed further.
     */
    if (mac->roam.neighborRoamInfo.neighborRoamState ==
                                          eCSR_NEIGHBOR_ROAM_STATE_INIT) {
        smsLog(mac, LOGE, FL("neighborRoamState %d"),
                          mac->roam.neighborRoamInfo.neighborRoamState);
        return eHAL_STATUS_FAILURE;
    }

    /*
     * Save the SME Session ID here. We need it while processing
     * the preauth response.
     */
    mac->ft.ftSmeContext.smeSessionId = session_id;
    vos_mem_zero(pre_auth_req, auth_req_len);

    pre_auth_req->pbssDescription = (tpSirBssDescription)vos_mem_malloc(
            sizeof(bss_description->length) + bss_description->length);
    if (NULL == pre_auth_req->pbssDescription) {
        smsLog(mac, LOGE,
               FL("Unable to allocate memory for preauth bss description"));
        return eHAL_STATUS_RESOURCES;
    }

    pre_auth_req->messageType =
                     pal_cpu_to_be16(eWNI_SME_MBB_PRE_AUTH_REASSOC_REQ);

    pre_auth_req->preAuthchannelNum = bss_description->channelId;

    vos_mem_copy((void *)&pre_auth_req->currbssId,
                 (void *)session->connectedProfile.bssid, sizeof(tSirMacAddr));
    vos_mem_copy((void *)&pre_auth_req->preAuthbssId,
                 (void *)bss_description->bssId, sizeof(tSirMacAddr));

    vos_mem_copy(pre_auth_req->pbssDescription, bss_description,
                 sizeof(bss_description->length) + bss_description->length);
    pre_auth_req->length = pal_cpu_to_be16(auth_req_len);

    /* Register mbb callback */
    smsLog(mac, LOG1, FL("Registering mbb callback"));
    csr_register_roaming_mbb_callback(mac);

    csrReleaseProfile(mac, &neighbor_roam_info->csrNeighborRoamProfile);

    /* Copy current profile to be used in csr_update_roamed_info_mbb */
    status = csrRoamCopyProfile(mac,
                                &neighbor_roam_info->csrNeighborRoamProfile,
                                session->pCurRoamProfile);
    if(!HAL_STATUS_SUCCESS(status)) {
       smsLog(mac, LOGE, FL("Profile copy failed"));
       return status;
    }

    return palSendMBMessage(mac->hHdd, pre_auth_req);
}

/**
 * csr_neighbor_roam_issue_preauth_reassoc() -issues  preauth_reassoc request
 * @mac: MAC context
 *
 * This function issues preauth_reassoc request to PE with the 1st AP
 * entry in the roamable AP list
 *
 * Return: eHAL_STATUS_SUCCESS on success, eHAL_STATUS_FAILURE otherwise
 */
eHalStatus csr_neighbor_roam_issue_preauth_reassoc(tpAniSirGlobal mac)
{
    tpCsrNeighborRoamControlInfo neighbor_roam_info =
                                           &mac->roam.neighborRoamInfo;
    eHalStatus status = eHAL_STATUS_SUCCESS;
    tpCsrNeighborRoamBSSInfo neighbor_bss_node;
    tCsrRoamInfo roam_info;

    VOS_ASSERT(neighbor_roam_info->FTRoamInfo.preauthRspPending ==
                                                         eANI_BOOLEAN_FALSE);

    neighbor_bss_node = csrNeighborRoamGetRoamableAPListNextEntry(mac,
                               &neighbor_roam_info->roamableAPList, NULL);

    if (neighbor_bss_node == NULL)
    {
        smsLog(mac, LOGE, FL("Roamable AP list is empty"));
        csrRoamOffloadScan(mac, ROAM_SCAN_OFFLOAD_RESTART,
                            REASON_NO_CAND_FOUND_OR_NOT_ROAMING_NOW);
        return eHAL_STATUS_FAILURE;
    }
    else
    {
        /*
         * Set is_preauth_lfr_mbb which will be checked in
         * different API's.
         */
        mac->ft.ftSmeContext.is_preauth_lfr_mbb = true;
        smsLog(mac, LOG1, FL("is_preauth_lfr_mbb %d"),
                mac->ft.ftSmeContext.is_preauth_lfr_mbb);

        status = csrRoamEnqueuePreauth(mac,
                 neighbor_roam_info->csrSessionId,
                 neighbor_bss_node->pBssDescription,
                 ecsr_mbb_perform_preauth_reassoc,
                 eANI_BOOLEAN_TRUE);

        smsLog(mac, LOG1, FL("Before Pre-Auth: BSSID "MAC_ADDRESS_STR", Ch:%d"),
               MAC_ADDR_ARRAY(neighbor_bss_node->pBssDescription->bssId),
               neighbor_bss_node->pBssDescription->channelId);

        if (eHAL_STATUS_SUCCESS != status)
        {
            smsLog(mac, LOGE,
                   FL("Send Preauth request to PE failed with status %d"),
                   status);
            return status;
        }
    }

    neighbor_roam_info->FTRoamInfo.preauthRspPending = eANI_BOOLEAN_TRUE;

    CSR_NEIGHBOR_ROAM_STATE_TRANSITION(eCSR_NEIGHBOR_ROAM_STATE_MBB_PREAUTH_REASSOC)

    if (csrRoamIsFastRoamEnabled(mac, CSR_SESSION_ID_INVALID))
    {
        smsLog(mac, LOG1, FL("Invoking eCSR_ROAM_PMK_NOTIFY"));
        vos_mem_copy((void *)&roam_info.bssid,
                     (void *)neighbor_bss_node->pBssDescription->bssId,
                     sizeof(tCsrBssid));
        csrRoamCallCallback(mac, neighbor_roam_info->csrSessionId, &roam_info,
                            0, eCSR_ROAM_PMK_NOTIFY, 0);
    }
    return status;
}

/**
 * csr_stop_preauth_reassoc_mbb_timer() -stops preauth_reassoc timer
 * @mac: MAC context
 *
 * This function stops preauth_reassoc timer
 *
 */
void csr_stop_preauth_reassoc_mbb_timer(tpAniSirGlobal mac)
{
    VOS_STATUS vos_status = VOS_STATUS_SUCCESS;

    if (mac->roam.neighborRoamInfo.is_pre_auth_reassoc_mbb_timer_started) {
        vos_status =
            vos_timer_stop(&mac->ft.ftSmeContext.pre_auth_reassoc_mbb_timer);
        mac->roam.neighborRoamInfo.is_pre_auth_reassoc_mbb_timer_started =
                                                                       false;
    }
}


/**
 * csr_preauth_reassoc_mbb_timer_callback() -preauth_reassoc timer callback
 * @mac: MAC context
 *
 * This function issues preauth_reassoc with another roamable entry
 *
 */
void csr_preauth_reassoc_mbb_timer_callback(void *context)
{
    tpAniSirGlobal mac = (tpAniSirGlobal)context;

    mac->roam.neighborRoamInfo.is_pre_auth_reassoc_mbb_timer_started = 0;

    smsLog(mac, LOG1, FL("is_pre_auth_reassoc_mbb_timer_started %d"),
           mac->roam.neighborRoamInfo.is_pre_auth_reassoc_mbb_timer_started);

    csr_neighbor_roam_issue_preauth_reassoc(mac);
}


/**
 * csr_roam_dequeue_preauth_reassoc() -Dequeues
 * ecsr_mbb_perform_preauth_reassoc
 * @mac: MAC context
 *
 * This function dequeues ecsr_mbb_perform_preauth_reassoc
 *
 */
eHalStatus csr_roam_dequeue_preauth_reassoc(tpAniSirGlobal mac)
{
    tListElem *entry;
    tSmeCmd *command;
    entry = csrLLPeekHead(&mac->sme.smeCmdActiveList, LL_ACCESS_LOCK);
    if (entry) {
        command = GET_BASE_ADDR(entry, tSmeCmd, Link);
        if ((eSmeCommandRoam == command->command) &&
            (ecsr_mbb_perform_preauth_reassoc ==
                                    command->u.roamCmd.roamReason)) {
            smsLog(mac, LOG1, FL("DQ-Command = %d, Reason = %d"),
                    command->command, command->u.roamCmd.roamReason);
            if (csrLLRemoveEntry( &mac->sme.smeCmdActiveList,
                                       entry, LL_ACCESS_LOCK)) {
                csrReleaseCommandPreauth( mac, command );
            }
        } else {
            smsLog(mac, LOGE, FL("Command = %d, Reason = %d "),
                    command->command, command->u.roamCmd.roamReason);
        }
    }
    else {
        smsLog(mac, LOGE,
               FL("pEntry NULL for eWNI_SME_MBB_PRE_AUTH_REASSOC_RSP"));
    }
    smeProcessPendingQueue( mac );
    return eHAL_STATUS_SUCCESS;
}

/**
 * csr_neighbor_roam_preauth_reassoc_rsp_handler() -handles preauth
 * reassoc response
 * @mac: MAC context
 * @lim_status: status of preauth reassoc response from lim
 * @bss_description: bss description pointer
 *
 * This function handles preauth_reassoc response from PE. When
 * preauth_reassoc response failure is received, preauth reassoc
 * with new candidate will be attempted. In success case, candidate will be
 * removed from roamable entry.
 *
 */
eHalStatus
csr_neighbor_roam_preauth_reassoc_rsp_handler(tpAniSirGlobal mac,
          tSirRetStatus lim_status, tSirBssDescription **bss_description)
{
    tpCsrNeighborRoamControlInfo neighbor_roam_info =
                                      &mac->roam.neighborRoamInfo;
    eHalStatus status = eHAL_STATUS_SUCCESS;
    eHalStatus preauth_processed = eHAL_STATUS_SUCCESS;
    tpCsrNeighborRoamBSSInfo preauth_rsp_node = NULL;

    if (eANI_BOOLEAN_FALSE ==
                neighbor_roam_info->FTRoamInfo.preauthRspPending) {
       /*
        * This can happen when we disconnect immediately after sending
        * a pre-auth request. During processing of the disconnect command,
        * we would have reset preauthRspPending and transitioned to INIT state.
        */
       smsLog(mac, LOGE,
              FL("Unexpected pre-auth response in state %d"),
              neighbor_roam_info->neighborRoamState);
       preauth_processed = eHAL_STATUS_FAILURE;
       goto DEQ_PREAUTH;
    }

    if ((neighbor_roam_info->neighborRoamState !=
                            eCSR_NEIGHBOR_ROAM_STATE_MBB_PREAUTH_REASSOC)) {
        smsLog(mac, LOGE,
               FL("Preauth response received in state %s"),
               macTraceGetNeighbourRoamState(
                      neighbor_roam_info->neighborRoamState));
        preauth_processed = eHAL_STATUS_FAILURE;
        goto DEQ_PREAUTH;
    }

    neighbor_roam_info->FTRoamInfo.preauthRspPending = eANI_BOOLEAN_FALSE;

    if (eSIR_SUCCESS == lim_status)
        preauth_rsp_node = csrNeighborRoamGetRoamableAPListNextEntry(mac,
                                  &neighbor_roam_info->roamableAPList, NULL);

    if ((eSIR_SUCCESS == lim_status) && (NULL != preauth_rsp_node)) {
        smsLog(mac, LOG1, FL("MBB Reassoc completed successfully"));

        smsLog(mac, LOG1, FL("After MBB reassoc BSSID "MAC_ADDRESS_STR" Ch %d"),
               MAC_ADDR_ARRAY(preauth_rsp_node->pBssDescription->bssId),
               preauth_rsp_node->pBssDescription->channelId);

        /* Memory will be freed in caller of this */
        *bss_description = (tpSirBssDescription)vos_mem_malloc(
                           sizeof(preauth_rsp_node->pBssDescription->length) +
                           preauth_rsp_node->pBssDescription->length);
        if (NULL == *bss_description) {
            smsLog(mac, LOGE,
                   FL("Unable to allocate memory for preauth bss description"));
            preauth_processed = eHAL_STATUS_RESOURCES;
            goto DEQ_PREAUTH;
        }

        vos_mem_copy(*bss_description, preauth_rsp_node->pBssDescription,
                     sizeof(preauth_rsp_node->pBssDescription->length) +
                     preauth_rsp_node->pBssDescription->length);

        /*
        * MBB Reassoc competer successfully. Insert the preauthenticated
        * node to tail of preAuthDoneList
        */
        csrNeighborRoamRemoveRoamableAPListEntry(mac,
                         &neighbor_roam_info->roamableAPList, preauth_rsp_node);
        csrLLInsertTail(&neighbor_roam_info->FTRoamInfo.preAuthDoneList,
                                  &preauth_rsp_node->List, LL_ACCESS_LOCK);
        return eHAL_STATUS_SUCCESS;
    } else {
        tpCsrNeighborRoamBSSInfo    neighbor_bss_node = NULL;
        tListElem                   *entry;

        /*
        * Pre-auth failed. Add the bssId to the preAuth failed list MAC Address.
        * Also remove the AP from roamable AP list. The one in the head of the
        * list should be one with which we issued pre-auth and failed.
        */
        entry = csrLLRemoveHead(&neighbor_roam_info->roamableAPList,
                                  LL_ACCESS_LOCK);
        if(entry) {
           neighbor_bss_node = GET_BASE_ADDR(entry,
                                            tCsrNeighborRoamBSSInfo, List);
           /*
            * Add the BSSID to pre-auth fail list if it is
            * not requested by HDD
            */
           status = csrNeighborRoamAddBssIdToPreauthFailList(mac,
                                 neighbor_bss_node->pBssDescription->bssId);

           smsLog(mac, LOG1,
               FL("MBB reassoc failed BSSID "MAC_ADDRESS_STR" Ch:%d status %d"),
               MAC_ADDR_ARRAY(neighbor_bss_node->pBssDescription->bssId),
               neighbor_bss_node->pBssDescription->channelId, lim_status);

           /* Now we can free this node */
           csrNeighborRoamFreeNeighborRoamBSSNode(mac, neighbor_bss_node);
        }

        /*
        * Move state to Connected. Connected state here signifies connection
        * with current AP as preauth failed with roamable AP. Still driver has
        * connection with current AP.
        */
        CSR_NEIGHBOR_ROAM_STATE_TRANSITION(eCSR_NEIGHBOR_ROAM_STATE_CONNECTED)

        /* Start a timer to issue preauth_reassoc request for the next entry*/
        status = vos_timer_start(&mac->ft.ftSmeContext.
                   pre_auth_reassoc_mbb_timer, PREAUTH_REASSOC_MBB_TIMER_VALUE);
        if (eHAL_STATUS_SUCCESS != status) {
            smsLog(mac, LOGE,
                   FL("pre_auth_reassoc_mbb_timer start failed status %d"),
                   status);
            preauth_processed = eHAL_STATUS_FAILURE;
            goto DEQ_PREAUTH;
        }
        mac->roam.neighborRoamInfo.is_pre_auth_reassoc_mbb_timer_started = true;
        smsLog(mac, LOG1, FL("is_pre_auth_reassoc_mbb_timer_started %d"),
           mac->roam.neighborRoamInfo.is_pre_auth_reassoc_mbb_timer_started);
        preauth_processed = eHAL_STATUS_SUCCESS;
    }

DEQ_PREAUTH:
    csr_roam_dequeue_preauth_reassoc(mac);
    return preauth_processed;
}

eHalStatus csr_update_roamed_info_mbb(tHalHandle hal,
     tpSirBssDescription bss_description, tpSirFTPreAuthRsp pre_auth_rsp)
{
    eHalStatus status;
    tpAniSirGlobal mac = PMAC_STRUCT(hal);
    tDot11fBeaconIEs *ies_local = NULL;
    tCsrRoamSession *session = NULL;
    tCsrRoamProfile *profile;
    tSirMacAddr broadcast_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    tANI_U32 key_timeout_interval;
    tCsrRoamInfo roam_info;
    sme_QosAssocInfo assoc_info;
    tANI_U32 len;
    tANI_U8 sme_session_id = pre_auth_rsp->smeSessionId;
    tANI_U8 acm_mask = 0;

    /* Get profile */
    session = CSR_GET_SESSION(mac, sme_session_id);

    profile = vos_mem_malloc(sizeof(*profile));
    if (NULL == profile) {
        smsLog(mac, LOGE, FL("Memory allocation failure for profile"));
        return eHAL_STATUS_FAILURE;
    }

    /*
     * session->pCurRoamProfile is copied into csrNeighborRoamProfile
     * in csr_roam_issue_preauth_reassoc_req as there is a chance of
     * session->pCurRoamProfile getting freed when disconnect is issued
     * from upper layer.
     */
    status = csrRoamCopyProfile(mac, profile,
                  &mac->roam.neighborRoamInfo.csrNeighborRoamProfile);
    if(!HAL_STATUS_SUCCESS(status)) {
       smsLog(mac, LOGE, FL("Profile copy failed"));
       vos_mem_free(profile);
       return status;
    }

    profile->negotiatedAuthType =
       mac->roam.roamSession[sme_session_id].connectedProfile.AuthType;
    profile->negotiatedUCEncryptionType =
       mac->roam.roamSession[sme_session_id].connectedProfile.EncryptionType;
    profile->mcEncryptionType =
       mac->roam.roamSession[sme_session_id].connectedProfile.mcEncryptionInfo;
    profile->negotiatedMCEncryptionType =
       mac->roam.roamSession[sme_session_id].connectedProfile.mcEncryptionType;

    smsLog(mac, LOG1,
           FL("AuthType %d UCEType %d Entries %d encryptionType %d MCEType %d"),
           profile->negotiatedAuthType,
           profile->negotiatedUCEncryptionType,
           profile->mcEncryptionType.numEntries,
           profile->mcEncryptionType.encryptionType[0],
           profile->negotiatedMCEncryptionType);

    /* Get IE's */
    status = csrGetParsedBssDescriptionIEs(mac, bss_description, &ies_local);
    if (!HAL_STATUS_SUCCESS(status)) {
        smsLog(mac, LOGE,
               FL("csrGetParsedBssDescriptionIEs failed"));
        vos_mem_free(profile);
        return status;
    }
    if(ies_local == NULL) {
       smsLog(mac, LOGE,
              FL("ies_local is NULL "));
       vos_mem_free(profile);
       return eHAL_STATUS_FAILURE;
    }

    vos_mem_set(&roam_info, sizeof(roam_info), 0);

    csrRoamStopNetwork(mac, sme_session_id, profile,
                                          bss_description, ies_local);

    session->connectState = eCSR_ASSOC_STATE_TYPE_INFRA_ASSOCIATED;

    csrRoamSaveConnectedInfomation(mac, sme_session_id, profile,
                                                  bss_description, ies_local);
    csrRoamSaveSecurityRspIE(mac, sme_session_id,
                             profile->negotiatedAuthType, bss_description,
                             ies_local);

    csrRoamStateChange(mac, eCSR_ROAMING_STATE_JOINED, sme_session_id);

    /*
     * If abortConnection is set, it implies that disconnect in progress
     * from upper layer. So there is no point in proceeding further.
     * By this point, pConnectBssDesc is updated in
     * csrRoamSaveConnectedInfomation. This pConnectBssDesc is used
     * during processing of disconnect in CSR and this has new BSSID info.
     * By this time, original AP related info was cleaned up in LIM. So,
     * disconnect queued while roaming in progress will take care of
     * cleaning up roamed AP related info in LIM.
     */
    if (session->abortConnection) {
        smsLog(mac, LOGE, FL("Disconnect in progress"));

        smsLog(mac, LOGE, FL("MBB reassoc BSSID "MAC_ADDRESS_STR" Ch %d"),
               MAC_ADDR_ARRAY(bss_description->bssId),
               bss_description->channelId);

        vos_mem_free(profile);
        return eHAL_STATUS_FAILURE;
    }

    if(CSR_IS_ENC_TYPE_STATIC(profile->negotiatedUCEncryptionType) &&
                                        !profile->bWPSAssociation) {
       /*
        * Issue the set Context request to LIM to establish the
        * Unicast STA context
        */
       if(!HAL_STATUS_SUCCESS(csrRoamIssueSetContextReq(mac,
                 sme_session_id,
                 profile->negotiatedUCEncryptionType,
                 bss_description, &(bss_description->bssId), FALSE, TRUE,
                 eSIR_TX_RX, 0, 0, NULL, 0))) {
          smsLog(mac, LOGE, FL("Set context for unicast fail"));
          csrRoamSubstateChange(mac, eCSR_ROAM_SUBSTATE_NONE,
                                        sme_session_id);
       }
       /*
        * Issue the set Context request to LIM to establish the
        * Broadcast STA context
        */
       csrRoamIssueSetContextReq(mac, sme_session_id,
                                 profile->negotiatedMCEncryptionType,
                                 bss_description, &broadcast_mac,
                                 FALSE, FALSE, eSIR_TX_RX, 0, 0, NULL, 0);
    } else if (!session->abortConnection) {
       /* Need to wait for supplicant authtication */
       roam_info.fAuthRequired = eANI_BOOLEAN_TRUE;
       /* Set the subestate to WaitForKey in case authentiation is needed */
       csrRoamSubstateChange(mac, eCSR_ROAM_SUBSTATE_WAIT_FOR_KEY,
                                            sme_session_id);

       if(profile->bWPSAssociation)
          key_timeout_interval = CSR_WAIT_FOR_WPS_KEY_TIMEOUT_PERIOD;
       else
          key_timeout_interval = CSR_WAIT_FOR_KEY_TIMEOUT_PERIOD;

       /* Save sessionId in case of timeout */
       mac->roam.WaitForKeyTimerInfo.sessionId = sme_session_id;
       /*
        * This time should be long enough for the rest of the
        * process plus setting key
        */
       if(!HAL_STATUS_SUCCESS(csrRoamStartWaitForKeyTimer(mac,
                                             key_timeout_interval))) {
          /* Reset our state so nothting is blocked */
          smsLog(mac, LOGE, FL("Failed to start pre-auth timer"));
          csrRoamSubstateChange(mac, eCSR_ROAM_SUBSTATE_NONE,
                                           sme_session_id);
       }
    }

    csrRoamFreeConnectedInfo(mac, &session->connectedInfo);

    assoc_info.pBssDesc = bss_description;
    assoc_info.pProfile = profile;

    len = pre_auth_rsp->roam_info->nBeaconLength +
          pre_auth_rsp->roam_info->nAssocReqLength +
          pre_auth_rsp->roam_info->nAssocRspLength;

    if(len) {
       session->connectedInfo.pbFrames = vos_mem_malloc(len);
       if (session->connectedInfo.pbFrames != NULL ) {
           vos_mem_copy(session->connectedInfo.pbFrames,
                        pre_auth_rsp->roam_info->pbFrames, len);
           session->connectedInfo.nAssocReqLength =
                           pre_auth_rsp->roam_info->nAssocReqLength;
           session->connectedInfo.nAssocRspLength =
                           pre_auth_rsp->roam_info->nAssocRspLength;
           session->connectedInfo.nBeaconLength =
                           pre_auth_rsp->roam_info->nBeaconLength;

           roam_info.nAssocReqLength = pre_auth_rsp->roam_info->nAssocReqLength;
           roam_info.nAssocRspLength = pre_auth_rsp->roam_info->nAssocRspLength;
           roam_info.nBeaconLength = pre_auth_rsp->roam_info->nBeaconLength;
           roam_info.pbFrames = session->connectedInfo.pbFrames;
       }
       session->connectedInfo.staId = pre_auth_rsp->roam_info->staId;
       roam_info.staId = pre_auth_rsp->roam_info->staId;
       roam_info.ucastSig = pre_auth_rsp->roam_info->ucastSig;
       roam_info.bcastSig = pre_auth_rsp->roam_info->bcastSig;
       roam_info.maxRateFlags = pre_auth_rsp->roam_info->maxRateFlags;
    }

    roam_info.statusCode = eSIR_SME_SUCCESS;
    roam_info.reasonCode = eSIR_SME_SUCCESS;
    roam_info.pBssDesc = bss_description;

    vos_mem_copy(&roam_info.bssid, &bss_description->bssId,
                                     sizeof(tCsrBssid));

    mac->roam.roamSession[sme_session_id].connectState =
                                     eCSR_ASSOC_STATE_TYPE_NOT_CONNECTED;

    sme_QosCsrEventInd(mac, sme_session_id,
                            SME_QOS_CSR_HANDOFF_ASSOC_REQ, NULL);
    sme_QosCsrEventInd(mac, sme_session_id, SME_QOS_CSR_REASSOC_REQ, NULL);
    sme_QosCsrEventInd(mac, sme_session_id, SME_QOS_CSR_HANDOFF_COMPLETE, NULL);

    mac->roam.roamSession[sme_session_id].connectState =
                                    eCSR_ASSOC_STATE_TYPE_INFRA_ASSOCIATED;
    sme_QosCsrEventInd(mac, sme_session_id,
                              SME_QOS_CSR_REASSOC_COMPLETE, &assoc_info);


    acm_mask = sme_QosGetACMMask(mac, bss_description, NULL);

    session->connectedProfile.acm_mask = acm_mask;
    if(session->connectedProfile.modifyProfileFields.uapsd_mask) {
       smsLog(mac, LOGE, "uapsd_mask (0x%X) set, request UAPSD now",
              session->connectedProfile.modifyProfileFields.uapsd_mask);
              pmcStartUapsd(mac, NULL, NULL);
    }
    session->connectedProfile.dot11Mode = session->bssParams.uCfgDot11Mode;
    roam_info.u.pConnectedProfile = &session->connectedProfile;

    if(!IS_FEATURE_SUPPORTED_BY_FW(SLM_SESSIONIZATION) &&
                      (csrIsConcurrentSessionRunning(mac)))
       mac->roam.configParam.doBMPSWorkaround = 1;

    csr_roam_dequeue_preauth_reassoc(mac);

    csrRoamCallCallback(mac, sme_session_id, &roam_info, 0,
             eCSR_ROAM_ASSOCIATION_COMPLETION, eCSR_ROAM_RESULT_ASSOCIATED);

    csrResetPMKIDCandidateList(mac, sme_session_id);
#ifdef FEATURE_WLAN_WAPI
    csrResetBKIDCandidateList(mac, sme_session_id);
#endif

    if(!CSR_IS_WAIT_FOR_KEY(mac, sme_session_id)) {
        smsLog(mac, LOG1, "NO CSR_IS_WAIT_FOR_KEY -> csr_roam_link_up");
        csrRoamLinkUp(mac, session->connectedProfile.bssid);
    }

    if (pmcShouldBmpsTimerRun(mac)) {
        if(eANI_BOOLEAN_TRUE == roam_info.fAuthRequired) {
           mac->pmc.full_power_till_set_key = true;
           smsLog(mac, LOG1,
                  FL("full_power_till_set_key is made true"));
        }

        if (pmcStartTrafficTimer(mac, BMPS_TRAFFIC_TIMER_ALLOW_SECURITY_DHCP)
                    != eHAL_STATUS_SUCCESS)
            smsLog(mac, LOGE, FL("Cannot start BMPS Retry timer"));

        smsLog(mac, LOG1, FL("BMPS Retry Timer already running or started"));
    }

    vos_mem_free(pre_auth_rsp->roam_info->pbFrames);
    vos_mem_free(pre_auth_rsp->roam_info);
    vos_mem_free(profile);
    vos_mem_free(ies_local);

    return eHAL_STATUS_SUCCESS;
}


/**
 * csr_roam_preauth_rsp_mbb_processor() -handles
 * eWNI_SME_MBB_PRE_AUTH_REASSOC_RSP
 * @hal: HAL context
 *
 * This function invokes preauth reassoc response handler and
 * updates CSR with new connection information.
 *
 */
void csr_roam_preauth_rsp_mbb_processor(tHalHandle hal,
     tpSirFTPreAuthRsp pre_auth_rsp)
{
    tpAniSirGlobal mac = PMAC_STRUCT(hal);
    eHalStatus  status;
    tCsrRoamInfo roam_info;
    tpSirBssDescription  bss_description = NULL;
    tCsrRoamSession *session = NULL;

    mac->ft.ftSmeContext.is_preauth_lfr_mbb = false;
    smsLog(mac, LOG1, FL("is_preauth_lfr_mbb %d"),
                         mac->ft.ftSmeContext.is_preauth_lfr_mbb);

    /*
    * When reason is SIR_MBB_DISCONNECTED, cleanup CSR info
    * of connected AP.
    */
    if (pre_auth_rsp->reason == SIR_MBB_DISCONNECTED) {
        /* Dequeue ecsr_perform_preauth_reassoc */
        csr_roam_dequeue_preauth_reassoc(mac);

        vos_mem_zero(&roam_info, sizeof(tCsrRoamInfo));
        csrRoamCallCallback(mac, pre_auth_rsp->smeSessionId, &roam_info, 0,
                               eCSR_ROAM_FT_REASSOC_FAILED, eSIR_SME_SUCCESS);

        csrRoamComplete(mac, eCsrJoinFailure, NULL);
    }

    status = csr_neighbor_roam_preauth_reassoc_rsp_handler(mac,
                                       pre_auth_rsp->status, &bss_description);
    if (status != eHAL_STATUS_SUCCESS) {
        smsLog(mac, LOGE,FL("Preauth was not processed: %d SessionID: %d"),
                            status, pre_auth_rsp->smeSessionId);
        /*
         * Preauth_reassoc is dequeued already in
         * csr_neighbor_roam_preauth_reassoc_rsp_handler for error cases.
         */
        return;
    }

    session = CSR_GET_SESSION(mac, pre_auth_rsp->smeSessionId);
    if (session->abortConnection) {
        smsLog(mac, LOGE,
               FL("Disconnect in progress, stop preauth/reassoc timer"));
        vos_timer_stop(&mac->ft.ftSmeContext.pre_auth_reassoc_mbb_timer);
    }

    /*
     * The below function calls/timers should be invoked only
     * if the pre-auth is successful. Also, Preauth_reassoc is dequeued
     * already in csr_neighbor_roam_preauth_reassoc_rsp_handler.
     */
    if (VOS_STATUS_SUCCESS != (VOS_STATUS)pre_auth_rsp->status)
        return;

    mac->ft.ftSmeContext.FTState = eFT_AUTH_COMPLETE;

    /* Save the received response */
    vos_mem_copy((void *)&mac->ft.ftSmeContext.preAuthbssId,
                 (void *)pre_auth_rsp->preAuthbssId, sizeof(tCsrBssid));


    /*
     * bss_description is updated in
     * csr_neighbor_roam_preauth_reassoc_rsp_handler
     */
    if (NULL == bss_description) {
        smsLog(mac, LOGE,
               FL("bss description is NULL"));
        goto DEQ_PREAUTH;
    }

    /* Update CSR for new connection */
    status = csr_update_roamed_info_mbb(hal, bss_description, pre_auth_rsp);
    /* In success case preauth reassoc is dequeued in
     * csr_update_roamed_info_mbb before updating HDD.
     */
    if(HAL_STATUS_SUCCESS(status))
       return;

DEQ_PREAUTH:
    csr_roam_dequeue_preauth_reassoc(mac);
}

/**
 * csr_prepare_reassoc_req () - Prepares reassoc request
 * @pmac: MAC context
 * @session_id: session id
 * @pbss_description: bss description
 * @preassoc_req: pointer to reassociation request
 *
 *Return: None
 */
static void csr_prepare_reassoc_req(void* pmac, tANI_U32 session_id,
       void* pbss_description, void *preassoc_req)
{
    tDot11fBeaconIEs *ies_local = NULL;
    eHalStatus status;
    tpAniSirGlobal mac = (tpAniSirGlobal)pmac;
    tpSirBssDescription bss_description = (tpSirBssDescription)pbss_description;
    tSirSmeJoinReq **reassoc_req = (tSirSmeJoinReq **)preassoc_req;

    /* Get IE's */
    status = csrGetParsedBssDescriptionIEs(mac, bss_description, &ies_local);
    if (!HAL_STATUS_SUCCESS(status)) {
        smsLog(mac, LOGE,
               FL("csrGetParsedBssDescriptionIEs failed"));
        return;
    }
    if(ies_local == NULL) {
       smsLog(mac, LOGE,
              FL("ies_local is NULL"));
       return;
    }

    smsLog(mac, LOG1, FL("session_id %d"), session_id);

    status = csr_fill_reassoc_req(mac, session_id, bss_description,
                         ies_local, reassoc_req);
    if (!HAL_STATUS_SUCCESS(status)) {
        smsLog(mac, LOGE,
               FL("Reassociation request filling failed"));
        return;
    }
    vos_mem_free(ies_local);
    smsLog(mac, LOG1, FL("status %d"), status);
}

/**
 * csr_roaming_mbb_callback () - handles mbb callback
 * @pmac: MAC context
 * @session_id: session id
 * @pbss_description: bss description
 * @preassoc_req: pointer to reassociation request
 * @csr_roam_op_code: callback opcode
 *
 *Return: None
 */
static void csr_roaming_mbb_callback(void* pmac, tANI_U32 session_id,
       void* pbss_description, void *preassoc_req, tANI_U32 csr_roam_op_code)
{
    tpAniSirGlobal mac = (tpAniSirGlobal)pmac;

    smsLog(mac, LOG1,
           FL("csr_roam_op_code %d"), csr_roam_op_code);

    switch(csr_roam_op_code) {
    case SIR_ROAMING_DEREGISTER_STA:
         csrRoamCallCallback(mac, session_id, NULL, 0,
                              eCSR_ROAM_FT_START, eSIR_SME_SUCCESS);
        break;
    case SIR_STOP_ROAM_OFFLOAD_SCAN:
         csrRoamOffloadScan(mac, ROAM_SCAN_OFFLOAD_STOP, REASON_DISCONNECTED);
       break;
    case SIR_PREPARE_REASSOC_REQ:
         csr_prepare_reassoc_req(pmac, session_id, pbss_description,
                         preassoc_req);
         break;
    }
}

/**
 * csr_register_roaming_mbb_callback () - registers roaming callback
 * @mac: MAC context
 *
 *Return: eHAL_STATUS_SUCCESS on success, otherwise  failure
 */
eHalStatus csr_register_roaming_mbb_callback(tpAniSirGlobal mac)
{
    eHalStatus status;

    status = sme_AcquireGlobalLock(&mac->sme);
    if (eHAL_STATUS_SUCCESS == status) {
        mac->sme.roaming_mbb_callback = csr_roaming_mbb_callback;
        sme_ReleaseGlobalLock(&mac->sme);
    } else
        smsLog(mac, LOGE,
               FL("sme_AcquireGlobalLock error"));
    return status;
}
