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

void lim_process_preauth_mbb_rsp_timeout(tpAniSirGlobal mac);
void lim_process_pre_auth_reassoc_req(tpAniSirGlobal mac, tpSirMsgQ msg);
void lim_handle_pre_auth_mbb_rsp(tpAniSirGlobal mac,
     tSirRetStatus status, tpPESession session_entry);
void lim_process_reassoc_mbb_rsp_timeout(tpAniSirGlobal mac);
void lim_handle_reassoc_mbb_fail(tpAniSirGlobal mac,
     tpPESession session_entry);
void lim_handle_reassoc_mbb_success(tpAniSirGlobal mac,
     tpPESession session_entry, tpSirAssocRsp  assoc_rsp, tpDphHashNode sta_ds);
void lim_process_sta_mlm_del_sta_rsp_mbb(tpAniSirGlobal mac,
     tpSirMsgQ lim_msg, tpPESession session_entry);
void lim_process_sta_mlm_del_bss_rsp_mbb(tpAniSirGlobal mac,
     tpSirMsgQ lim_msg, tpPESession session_entry);
void lim_process_sta_mlm_add_bss_rsp_mbb(tpAniSirGlobal mac,
     tpSirMsgQ limMsgQ,tpPESession session_entry);
void lim_process_sta_mlm_add_sta_rsp_mbb(tpAniSirGlobal mac,
     tpSirMsgQ limMsgQ,tpPESession session_entry);
eAniBoolean lim_is_mbb_reassoc_in_progress(tpAniSirGlobal mac,
     tpPESession session_entry);
