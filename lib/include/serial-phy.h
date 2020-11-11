/*
 * Copyright (c) 2017 Sebastian Boehm (BTU-CS)
 *
 * @brief 	IEEE 802.15.4 PHY service for connection to simulated 802.15.4
 * 			According to IEEE 802.15.4-2006 specification
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

//#include "contiki.h"
#include "phy.h"
//#include "pcapng.h"
//#include "pcapng-line.h"

#ifndef __SERIAL_PHY_H__
#define __SERIAL_PHY_H__

/** ------- PHY Data Service Primitive Header Lengths --------- */

/** @brief Maximum PHY message header size */
#define maxPHYMessageHeaderSize 4;

/** @brief Size in bytes of PD_DATA_REQUEST message header */
#define SIZEOF_PD_DATA_REQUEST					3

/** @brief Size in bytes of PD_DATA_CONFIRM message header */
#define SIZEOF_PD_DATA_CONFIRM					3

/** @brief Size in bytes of PD_DATA_INDICATION message header */
#define SIZEOF_PD_DATA_INDICATION				4

/** @brief Size in bytes of PLME_CCA_REQUEST message header */
#define SIZEOF_PLME_CCA_REQUEST					2

/** @brief Size in bytes of PLME_CCA_CONFIRM message header */
#define SIZEOF_PLME_CCA_CONFIRM					3

/** @brief Size in bytes of PLME_ED_REQUEST message header */
#define SIZEOF_PLME_ED_REQUEST					2

/** @brief Size in bytes of PLME_ED_CONFIRM message header */
#define SIZEOF_PLME_ED_CONFIRM					4

/** @brief Size in bytes of PLME_GET_REQEST message header */
#define SIZEOF_PLME_GET_REQEST					3

/** @brief Size in bytes of PLME_GET_CONFIRM message header */
#define SIZEOF_PLME_GET_CONFIRM					4

/** @brief Size in bytes of PLME_SET_TRX_STATE_REQUEST message header */
#define SIZEOF_PLME_SET_TRX_STATE_REQUEST		3

/** @brief Size in bytes of PLME_SET_TRX_STATE_CONFIRM message header */
#define SIZEOF_PLME_SET_TRX_STATE_CONFIRM		3

/** @brief Size in bytes of PLME_SET_REQUEST message header */
#define SIZEOF_PLME_SET_REQUEST					3

/** @brief Size in bytes of PLME_SET_CONFIRM message header */
#define SIZEOF_PLME_SET_CONFIRM					4

/** @brief Size in bytes of PLME_SET_CONFIRM message header */
#define SIZEOF_RF_INDICATION					3

/** -------------- PHY Data Service Primitives --------------- */

/** @brief pd data request */
typedef struct pd_data_req
{
    uint8_t psduLength;       			/**< encapsulated payload length*/
    uint8_t data[aMaxPHYPacketSize];	/**< payload byte sequence */
} pd_data_req;

/** @brief pd data confirm */
typedef struct pd_data_conf
{
	phy_state status;					/**< PHY state */
} pd_data_conf;

/** @brief pd data indication */
typedef struct pd_data_ind
{
	uint8_t psduLength;       			/**< encapsulated payload length*/
	uint8_t ppduLinkQuality;  			/**< link quality of the PPDU */
	uint8_t data[aMaxPHYPacketSize];	/**< payload byte sequence */
} pd_data_ind;

/** ----------- PHY Management Service Primitives ------------ */

/** @brief plme cca request */
typedef struct plme_cca_req
{
} plme_cca_req;

//typedef plme_cca_req;

/** @brief plme cca confirm */
typedef struct plme_cca_conf
{
	phy_state status;					/**< PHY state */
} plme_cca_conf;

/** --------------------------- */

/** @brief plme ed request */
typedef struct plme_ed_req
{
} plme_ed_req;

//typedef plme_ed_req;

/** @brief plme ed confirm */
typedef struct plme_ed_conf
{
	phy_state status;					/**< PHY state */
    uint8_t energyLevel;				/**< ED energy level */
} plme_ed_conf;

/** --------------------------- */

/** @brief plme get request */
typedef struct plme_get_req
{
	phy_pib_attr attribute;				/**< PHY PIB attribute identifier */
} plme_get_req;

/** @brief plme get confirm */
typedef struct plme_get_conf
{
	phy_state status;					/**< PHY state */
	phy_pib_attr attribute;				/**< PHY PIB attribute identifier */
	PhyPIB_value value;					/**< specific PIB attribute value */
} plme_get_conf;

/** --------------------------- */

/** @brief plme set trx state request */
typedef struct plme_set_trx_state_req
{
	phy_state status;					/**< PHY state */
} plme_set_trx_state_req;

/** @brief plme set trx state request */
typedef struct plme_set_trx_state_conf
{
	phy_state status;					/**< PHY state */
} plme_set_trx_state_conf;

/** --------------------------- */

/** @brief plme set request */
typedef struct plme_set_req
{
	phy_pib_attr attribute;				/**< PHY PIB attribute identifier */
    PhyPIB_value value;					/**< specific PIB attribute value */
} plme_set_req;

/** @brief plme set confirm */
typedef struct plme_set_conf
{
	phy_state status;					/**< PHY state */
	phy_pib_attr attribute;				/**< PHY PIB attribute identifier */
} plme_set_conf;

/** --------------- PHY RF Service Primitives ---------------- */

/** @brief plme cca request */
typedef struct rf_ind
{
	uint8_t ppduLength;					/**< encapsulated payload length*/
	uint8_t data[aMaxPHYPacketSize];	/**< payload byte sequence */
} rf_ind;

/** --------------------------- */

typedef struct plme_set_phy_pib_conf
{
    phy_state status;
    uint8_t currChanAttr;
    uint32_t chanSupAttr;
    uint8_t trPwrAttr;
    uint8_t cCAModeAttr;
    uint8_t CurrPageAttr;
    uint16_t maxFrDurAttr;
    uint8_t shdrDurAttr;
    uint8_t symOcAttr;
    uint8_t LQI;
    uint8_t txgain;
    uint8_t rxgain;
    uint32_t bandwidth;
    uint32_t sampling_rate;
	uint8_t signalstrength;
    //letzten ED Wert hinzufügen.
}plme_set_phy_pib_conf;

typedef struct plme_set_phy_pib_req
{
	uint8_t currChanAttr;
	uint32_t chanSupAttr;
	uint8_t trPwrAttr;
	uint8_t cCAModeAttr;
	uint8_t CurrPageAttr;
	uint16_t maxFrDurAttr;
	uint8_t shdrDurAttr;
	uint8_t symOcAttr;
	uint8_t LQI;
	uint8_t txgain;
	uint8_t rxgain;
	uint32_t bandwidth;
	uint32_t sampling_rate;
	uint8_t signalstrength;
}plme_set_phy_pib_req;


typedef struct plme_set_phy_pib_ind
{
    phy_state status;
    uint8_t currChanAttr;
    uint32_t chanSupAttr;
    uint8_t trPwrAttr;
    uint8_t cCAModeAttr;
    uint8_t CurrPageAttr;
    uint16_t maxFrDurAttr;
    uint8_t shdrDurAttr;
    uint8_t symOcAttr;
    uint8_t LQI;
    uint8_t txgain;
    uint8_t rxgain;
    uint32_t bandwidth;
    uint32_t sampling_rate;
    uint8_t signalstrength;
}plme_set_phy_pib_ind;

/** @brief Union capable of holding any specific PHY messages */
typedef union
{
	pd_data_req data_req;						/**< PHY_DATA_req */
	pd_data_conf data_conf;						/**< PHY_DATA_conf */
	pd_data_ind data_ind;						/**< PHY_DATA_ind */
	plme_cca_req cca_req;						/**< PHY_CCA_req */
	plme_cca_conf cca_conf;						/**< PHY_CCA_conf */
	plme_ed_req ed_req;							/**< PHY_ED_req */
	plme_ed_conf ed_conf;						/**< PHY_ED_conf */
	plme_get_req get_req;						/**< PHY_GET_req */
	plme_get_conf get_conf;						/**< PHY_GET_conf */
	plme_set_trx_state_req set_trx_state_req;	/**< PHY_SET_TRX_STATE_req */
	plme_set_trx_state_conf set_trx_state_conf;	/**< PHY_SET_TRX_STATE_conf */
	plme_set_req set_req;						/**< PHY_SET_TRX_STATE_req */
	plme_set_conf set_conf;						/**< PHY_SET_TRX_STATE_conf */
    plme_set_phy_pib_conf pib_conf;
    plme_set_phy_pib_req pib_req;
    plme_set_phy_pib_ind pib_ind;
} PHY_msg_x;

/** @brief Compound PHY message */
typedef struct
{
	msg_type_sap type;		/**< PHY message type, e.g. PD_DATA_REQUEST */
	uint8_t length;			/**< PHY message length */
	PHY_msg_x x;			/**< specific PHY message */
} PHY_msg;

int serialize_msg(PHY_msg * msg, uint8_t buffer[aMaxPHYPacketSize]);
int deserialize_msg(uint8_t *stream, PHY_msg *msg);
void send_msg(PHY_msg * msg);
void print_msg_payload(void *data, uint8_t length, char *info);
void print_msg(PHY_msg * msg, char *info);
void send_test(void);

#endif /* __SERIAL_PHY_H__ */
