/*
 * Linux cfg80211 Vendor Extension Code
 *
 * $Copyright Open Broadcom Corporation$
 *
 * $Id: wl_cfgvendor.h 455257 2014-02-13 08:10:24Z chanyun $
 */

/*
 * New vendor interface additon to nl80211/cfg80211 to allow vendors
 * to implement proprietary features over the cfg80211 stack.
 */

#ifndef _wl_cfgvendor_h_
#define _wl_cfgvendor_h_

#define OUI_BRCM  0x001018
#define BRCM_VENDOR_SUBCMD_PRIV_STR	1

enum wl_vendor_subcmd {
	BRCM_VENDOR_SCMD_UNSPEC,
	BRCM_VENDOR_SCMD_PRIV_STR
};

enum wl_vendor_event {
	BRCM_VENDOR_EVENT_UNSPEC,
	BRCM_VENDOR_EVENT_PRIV_STR
};

/* Capture the BRCM_VENDOR_SUBCMD_PRIV_STRINGS* here */
#define BRCM_VENDOR_SCMD_CAPA	"cap"

#if defined(WL_VENDOR_EXT_SUPPORT)
extern int wl_cfgvendor_attach(struct wiphy *wiphy);
extern int wl_cfgvendor_detach(struct wiphy *wiphy);
#endif /* defined(WL_VENDOR_EXT_SUPPORT) */

#endif /* _wl_cfgvendor_h_ */
