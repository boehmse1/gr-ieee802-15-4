/* -*- c++ -*- */
/*
 * Copyright 2020 semulate.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "phy_msg_handler_impl.h"

#define dout d_debug && std::cout

namespace gr {
  namespace ieee802_15_4 {

    phy_msg_handler::sptr
    phy_msg_handler::make(uint64_t center_freq0,uint32_t samp_rate, uint32_t bw0, uint8_t txg, uint8_t rxg, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new phy_msg_handler_impl(center_freq0,samp_rate,bw0,txg,rxg, debug));
    }

    /*
     * The private constructor
     */
    phy_msg_handler_impl::phy_msg_handler_impl(uint64_t center_freq0,uint32_t samp_rate, uint32_t bw0, uint8_t txg, uint8_t rxg, bool debug)
      : gr::block("phy_msg_handler",
                  gr::io_signature::make(0, 0, 0),
                  gr::io_signature::make(0, 0, 0)),
        center_freq(center_freq0),sampling_rate(samp_rate),bandwidth(bw0),txgain(txg),rxgain(rxg),d_debug(debug)
    {

        message_port_register_in(pmt::mp("tcp_in"));
        message_port_register_in(pmt::mp("phy_in"));
        message_port_register_in(pmt::mp("signal_strength"));
        message_port_register_out(pmt::mp("manag_tx_out"));
        message_port_register_out(pmt::mp("manag_rx_out"));
        message_port_register_out(pmt::mp("data_out"));
        message_port_register_out(pmt::mp("conf_out"));
        set_msg_handler(pmt::mp("tcp_in"),boost::bind(&phy_msg_handler_impl::handle_pdu,this,_1));
        set_msg_handler(pmt::mp("phy_in"),boost::bind(&phy_msg_handler_impl::handle_phymsg,this,_1));
        set_msg_handler(pmt::mp("signal_strength"),boost::bind(&phy_msg_handler_impl::handle_signalmsg,this,_1));
        phy_msg_handler_impl::setCenter_freq(center_freq);
        phy_msg_handler_impl::setCCAModeAttr(1);// ed mit EDThreshold.
        phy_msg_handler_impl::setChanSupAttr(0);
        phy_msg_handler_impl::setCurrPageAttr(0);
        phy_msg_handler_impl::setLQI(125);
        phy_msg_handler_impl::setMaxFrDurAttr(0);
        phy_msg_handler_impl::setShdrDurAttr(10);// # 5 Bytes (1 Byte Preamble + 1 Byte SFD) = 10 Symbols @ 2.4 GHz O-QPSK
        phy_msg_handler_impl::setSymOcAttr(2);//  # 2 symbols for 1 Byte @ 2.4 GHz O-QPSK
        phy_msg_handler_impl::setTrPwrAttr(0);
        phy_msg_handler_impl::setSignalstrength(0);
        phy_msg_handler_impl::setCca_threshold(-10);


//      ED&CCA Variablen
        signalstrenghtcounter=0;
        signalstrength=0;
        cca= false;
        ed= false;
//      Initialisierungsvariable
        start=false;
    }

    /*
     * Our virtual destructor.
     */
    phy_msg_handler_impl::~phy_msg_handler_impl()
    {
    }


    void
    phy_msg_handler_impl::handle_pdu(pmt::pmt_t pdu)
    {
        pmt::pmt_t meta(pmt::car(pdu));
        pmt::pmt_t input(pmt::cdr(pdu));
        pmt::pmt_t output;

        dout << "PHY_msg_handler: blob length: " << pmt::blob_length(input) << std::endl;
        uint8_t *ptr = (uint8_t *) pmt::blob_data(input);
        PHY_msg msg = {};

        int i = phy_msg_handler_impl::deserialize_msg(ptr, &msg);
        dout << "PHY_msg_handler: handle message type: " << msgTypeToString(msg.type) << std::endl;
        phy_msg_handler_impl::handle_msg(&msg);
    }

    void
    phy_msg_handler_impl::handle_phymsg(pmt::pmt_t pdu)
    {
      if(start) {
          if (packet_couner < 0) {
              packet_couner++;
          } else {
              packet_couner = 0;
              pmt::pmt_t meta(pmt::car(pdu));
              pmt::pmt_t input(pmt::cdr(pdu));
              dout << "PHY_msg_handler: got PHY PDU: " << pmt::blob_length(input) << std::endl;
              assert(pmt::is_dict(meta));
              uint64_t x = 1;

              x = pmt::to_uint64(pmt::dict_ref(meta, pmt::mp("lqi"), pmt::from_long(x)));
              dout << "PHY_msg_handler: got Packet LQI: " << x << std::endl;
              if (x >= 0 && x <= 255) {
                  uint8_t y;
                  y = static_cast<uint8_t>(x);
                  phy_msg_handler_impl::setLQI(y);
              } else {
                  phy_msg_handler_impl::setLQI(0);
              }

              PHY_msg conf={};
              conf.type = PD_DATA_INDICATION;
              conf.x.data_ind.ppduLinkQuality = phy_msg_handler_impl::getLQI();
              conf.x.data_ind.psduLength = pmt::blob_length(input);
              uint8_t *ptr = (uint8_t *) pmt::blob_data(input);
              memcpy(conf.x.data_ind.data, ptr, pmt::blob_length(input));
              conf.length = sizeof(conf.type) + sizeof(conf.x.data_ind.ppduLinkQuality) +
                            sizeof(conf.x.data_ind.psduLength) + conf.x.data_ind.psduLength;
              phy_msg_handler_impl::send_conf(conf);
              PHY_msg conf2={};
              phy_msg_handler_impl::get_PIB_attrs(&conf2,true);
              phy_msg_handler_impl::send_conf(conf2);
          }
      }else{
          dout << "PHY_msg_handler: Initialisierung noch nicht Abgeschlossen, empfangenes Paket verworfen!"<<std::endl;
      }
    }

    void
    phy_msg_handler_impl::handle_msg(PHY_msg *msg)
    {
        dout << "PHY_msg_handler: handle message " << std::endl;
    PHY_msg conf={};
    PHY_msg PIBupdate={};
    pmt::pmt_t  output;
    pmt::pmt_t  payload;
    if(start) {
        dout << "PHY_msg_handler: handle message " << msgTypeToString(msg->type) << std::endl;
      switch (msg->type) {
          case PLME_SET_PHY_PIB_REQUEST:
              phy_msg_handler_impl::setCCAModeAttr(msg->x.pib_req.cCAModeAttr);
              phy_msg_handler_impl::setMaxFrDurAttr(msg->x.pib_req.maxFrDurAttr);
              phy_msg_handler_impl::setCurrChanAttr(msg->x.pib_req.currChanAttr);
              phy_msg_handler_impl::setSampling_rate(msg->x.pib_req.sampling_rate);
              phy_msg_handler_impl::setCurrPageAttr(msg->x.pib_req.CurrPageAttr);
              phy_msg_handler_impl::setTxgain(msg->x.pib_req.txgain);
              phy_msg_handler_impl::setRxgain(msg->x.pib_req.rxgain);
              phy_msg_handler_impl::setBandwidth(msg->x.pib_req.bandwidth);
              phy_msg_handler_impl::setLQI(msg->x.pib_req.LQI);
              phy_msg_handler_impl::setTrPwrAttr(msg->x.pib_req.trPwrAttr);
              phy_msg_handler_impl::setSymOcAttr(msg->x.pib_req.symOcAttr);
              phy_msg_handler_impl::setChanSupAttr(msg->x.pib_req.chanSupAttr);
              phy_msg_handler_impl::setShdrDurAttr(msg->x.pib_req.shdrDurAttr);
              phy_msg_handler_impl::setSignalstrength(msg->x.pib_req.signalstrength);
              phy_msg_handler_impl::get_PIB_attrs(&conf,false);
              phy_msg_handler_impl::send_conf(conf);
              break;
          case PD_DATA_REQUEST:
              msg->x.data_req.data;
              payload = pmt::make_blob(msg->x.data_req.data, msg->x.data_req.psduLength);
              output = pmt::cons(pmt::PMT_NIL, payload);
              message_port_pub(pmt::mp("data_out"), output);
              conf.type = PD_DATA_CONFIRM;
              conf.x.data_conf.status = phy_SUCCESS;
              conf.length = SIZEOF_PD_DATA_CONFIRM;
              phy_msg_handler_impl::send_conf(conf);
              break;
          case PLME_SET_REQUEST: {
              dout << "PHY_msg_handler: attribute: " << phyAttrToString(msg->x.set_req.attribute) << std::endl;
              switch (msg->x.set_req.attribute) {
                  case phyCurrentChannel: {
                      short k = 0;
                      k = (msg->x.set_req.value.currentChannel);
                      phy_msg_handler_impl::setCurrChanAttr(k);
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyCurrentChannel;
                      conf.x.set_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
                  case phyChannelsSupported:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyChannelsSupported;
                      conf.x.set_conf.status = phy_UNSUPPORT_ATTRIBUTE;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phyTransmitPower:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyTransmitPower;
                      conf.x.set_conf.status = phy_UNSUPPORT_ATTRIBUTE;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phyCCAMode:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyCCAMode;
                      conf.x.set_conf.status = phy_UNSUPPORT_ATTRIBUTE;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phyCurrentPage:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyCurrentPage;
                      conf.x.set_conf.status = phy_READ_ONLY;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::setCurrPageAttr(msg->x.set_req.value.currentPage);
                      break;
                  case phyMaxFrameDuration:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyMaxFrameDuration;
                      conf.x.set_conf.status = phy_UNSUPPORT_ATTRIBUTE;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::setMaxFrDurAttr(msg->x.set_req.value.maxFrameDuration);
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phySHRDuration:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phySHRDuration;
                      conf.x.set_conf.status = phy_UNSUPPORT_ATTRIBUTE;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phySymbolsPerOctet:
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phySymbolsPerOctet;
                      conf.x.set_conf.status = phy_UNSUPPORT_ATTRIBUTE;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phyRxGain: {
                      uint8_t k = (msg->x.set_req.value.RxGain);
                      phy_msg_handler_impl::setRxgain(k);
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyRxGain;
                      conf.x.set_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
                  case phyTxGain: {
                      uint8_t j = (msg->x.set_req.value.TxGain);
                      phy_msg_handler_impl::setTxgain(j);
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyTxGain;
                      conf.x.set_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
                  case phyBandwidth: {
                      phy_msg_handler_impl::setBandwidth(msg->x.set_req.value.Bandwidth);
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyBandwidth;
                      conf.x.set_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
                  case phySamplingRate: {
                      phy_msg_handler_impl::setSampling_rate(msg->x.set_req.value.SamplingRate);
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phySamplingRate;
                      conf.x.set_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
                  case phyLQI: {
                      phy_msg_handler_impl::setLQI(msg->x.set_req.value.LQI);
                      conf.type = PLME_SET_CONFIRM;
                      conf.x.set_conf.attribute = phyLQI;
                      conf.x.set_conf.status = phy_READ_ONLY;
                      conf.length = SIZEOF_PLME_SET_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
              }
              phy_msg_handler_impl::get_PIB_attrs(&PIBupdate,true);
              phy_msg_handler_impl::send_conf(PIBupdate);
              break;
          }
          case PLME_SET_TRX_STATE_REQUEST: {
              dout << "PHY_msg_handler: status: " << phyStateToString(msg->x.set_trx_state_req.status) << std::endl;
              switch (msg->x.set_trx_state_req.status) {
                  default:
                      conf.type = PLME_SET_TRX_STATE_CONFIRM;
                      conf.x.set_trx_state_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_TRX_STATE_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  case phy_RX_ON: {
                      phy_msg_handler_impl::setTxgain(0);
                      phy_msg_handler_impl::setRxgain(60);
                      conf.type = PLME_SET_TRX_STATE_CONFIRM;
                      conf.x.set_trx_state_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_TRX_STATE_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
                  case phy_TX_ON: {
                      phy_msg_handler_impl::setRxgain(0);
                      phy_msg_handler_impl::setTxgain(90);
                      conf.type = PLME_SET_TRX_STATE_CONFIRM;
                      conf.x.set_trx_state_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_TRX_STATE_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);

                      break;
                  }
                  case phy_FORCE_TRX_OFF:
                  case phy_TRX_OFF: {
                      phy_msg_handler_impl::setTxgain(0);
                      phy_msg_handler_impl::setRxgain(0);
                      conf.type = PLME_SET_TRX_STATE_CONFIRM;
                      conf.x.set_trx_state_conf.status = phy_SUCCESS;
                      conf.length = SIZEOF_PLME_SET_TRX_STATE_CONFIRM;
                      phy_msg_handler_impl::send_conf(conf);
                      break;
                  }
              }
              // phy_msg_handler_impl::get_PIB_attrs(&PIBupdate);
              // phy_msg_handler_impl::send_conf(PIBupdate);
              break;
          }

          case PLME_CCA_REQUEST: {
              signalstrenghtcounter = 0;
              signalstrength = -90.0;
              cca = true;
              ed = true;
              break;
          }
          case PLME_ED_REQUEST: {
              signalstrenghtcounter = 0;
              signalstrength = -90.0;
              ed = true;
              cca = false;
              break;
          }
      }
    }else{
        dout << "PHY_msg_handler: INIT message " << msgTypeToString(msg->type) << std::endl;
      switch (msg->type) {
          case PLME_SET_PHY_PIB_REQUEST:{
              start=true;
              phy_msg_handler_impl::setCCAModeAttr(msg->x.pib_req.cCAModeAttr);
              phy_msg_handler_impl::setMaxFrDurAttr(msg->x.pib_req.maxFrDurAttr);
              phy_msg_handler_impl::setCurrChanAttr(msg->x.pib_req.currChanAttr);
              phy_msg_handler_impl::setSampling_rate(msg->x.pib_req.sampling_rate);
              phy_msg_handler_impl::setCurrPageAttr(msg->x.pib_req.CurrPageAttr);
              phy_msg_handler_impl::setTxgain(msg->x.pib_req.txgain);
              phy_msg_handler_impl::setRxgain(msg->x.pib_req.rxgain);
              phy_msg_handler_impl::setBandwidth(msg->x.pib_req.bandwidth);
              phy_msg_handler_impl::setLQI(msg->x.pib_req.LQI);
              phy_msg_handler_impl::setTrPwrAttr(msg->x.pib_req.trPwrAttr);
              phy_msg_handler_impl::setSymOcAttr(msg->x.pib_req.symOcAttr);
              phy_msg_handler_impl::setChanSupAttr(msg->x.pib_req.chanSupAttr);
              phy_msg_handler_impl::setShdrDurAttr(msg->x.pib_req.shdrDurAttr);
              phy_msg_handler_impl::setSignalstrength(msg->x.pib_req.signalstrength);
              phy_msg_handler_impl::get_PIB_attrs(&conf, false);
              phy_msg_handler_impl::send_conf(conf);
              break;
          }
          default:{
              dout << "PHY_msg_handler: INIT message nicht " << msgTypeToString(PLME_SET_PHY_PIB_REQUEST) <<std::endl;
              break;
          }
      }

    }
    }

    void
    phy_msg_handler_impl::handle_signalmsg(pmt::pmt_t pdu)
    {
    phy_msg_handler_impl::setSignalstrength(pmt::to_double(pdu));
    if(signalstrenghtcounter>5 && ed){
        dout << "PHY_msg_handler: in signalstrength > 5"<<std::endl;
      ed=false;
      if(!cca){
          dout << "PHY_msg_handler: in ed"<<std::endl;
          performED();
      }else{
          dout << "PHY_msg_handler: in cca"<<std::endl;
          performCCA();
      }
    }
    if (ed){
      signalstrength=pmt::to_double(pdu) > signalstrength ? pmt::to_double(pdu):signalstrength;
      signalstrenghtcounter++;
    }
    }

    void
    phy_msg_handler_impl::performCCA()
    {
        dout << "PHY_msg_handler: perform CCA" << std::endl;
      PHY_msg conf;
      if(getSignalstrength()>=getCca_threshold()){
          conf.x.cca_conf.status=phy_BUSY;
      }else{
          conf.x.cca_conf.status=phy_IDLE;
      }
      conf.type=PLME_CCA_CONFIRM;
      conf.length=SIZEOF_PLME_CCA_CONFIRM;
      phy_msg_handler_impl::send_conf(conf);
    }

    void phy_msg_handler_impl::performED() {
        dout << "PHY_msg_handler: perform ED" << std::endl;
      PHY_msg conf={};
      double energy;
    //      Messwert wird über lineare Funktion auf Wertebereich 0 ... 255 angepasst.
      energy=(uint8_t)((11.6*getSignalstrength())+244);
      energy= energy < 255 ? energy: 255;
      conf.type=PLME_ED_CONFIRM;
      conf.x.ed_conf.status=phy_SUCCESS;
      conf.x.ed_conf.energyLevel=(uint8_t)energy;
      conf.length=SIZEOF_PLME_ED_CONFIRM;
      phy_msg_handler_impl::send_conf(conf);
    }

    void
    phy_msg_handler_impl::send_conf(PHY_msg conf)
    {
      pmt::pmt_t  output;
      uint8_t size = conf.length;
      uint8_t buffer[size];
      memset(&buffer, 0, sizeof(buffer));
      serializes_msg(&conf, buffer);
      output = pmt::make_blob(buffer, size);
      message_port_pub(pmt::mp("conf_out"), pmt::cons(pmt::PMT_NIL, output));
        dout << "PHY_msg_handler: send" << msgTypeToString(conf.type) <<std::endl;
      if(conf.type==PLME_SET_PHY_PIB_INDICATION){
          dout << "PHY_msg_handler: -----------------------------------" << msgTypeToString(conf.type) <<std::endl;
      }
    }

    void
    phy_msg_handler_impl::get_PIB_attrs(PHY_msg *conf, bool ind)
    {
      if(ind){
          conf->type=PLME_SET_PHY_PIB_INDICATION;
          conf->x.pib_ind.status = phy_SUCCESS;
          conf->x.pib_ind.cCAModeAttr = phy_msg_handler_impl::getCCAModeAttr();
          conf->x.pib_ind.chanSupAttr = 0;
          conf->x.pib_ind.currChanAttr = phy_msg_handler_impl::getCurrChanAttr();
          conf->x.pib_ind.CurrPageAttr = phy_msg_handler_impl::getCurrPageAttr();
          conf->x.pib_ind.LQI = phy_msg_handler_impl::getLQI();
          conf->x.pib_ind.maxFrDurAttr = phy_msg_handler_impl::getMaxFrDurAttr();
          conf->x.pib_ind.shdrDurAttr = phy_msg_handler_impl::getShdrDurAttr();
          conf->x.pib_ind.symOcAttr = phy_msg_handler_impl::getSymOcAttr();
          conf->x.pib_ind.trPwrAttr = phy_msg_handler_impl::getTrPwrAttr();
          conf->x.pib_ind.rxgain = phy_msg_handler_impl::getRxgain();
          conf->x.pib_ind.txgain = phy_msg_handler_impl::getTxgain();
          conf->x.pib_ind.bandwidth = phy_msg_handler_impl::getBandwidth();
          conf->x.pib_ind.sampling_rate = phy_msg_handler_impl::getSampling_rate();
          conf->x.pib_ind.signalstrength = phy_msg_handler_impl::getSignalstrength();
          conf->length =
                  sizeof(conf->type) + sizeof(conf->x.pib_ind.cCAModeAttr) + sizeof(conf->x.pib_ind.chanSupAttr)
                  + sizeof(conf->x.pib_ind.currChanAttr) + sizeof(conf->x.pib_ind.CurrPageAttr) +
                  sizeof(conf->x.pib_ind.LQI)
                  + sizeof(conf->x.pib_ind.maxFrDurAttr) + sizeof(conf->x.pib_ind.shdrDurAttr) +
                  sizeof(conf->x.pib_ind.symOcAttr)
                  + sizeof(conf->x.pib_ind.trPwrAttr) + sizeof(conf->x.pib_ind.status) +
                  sizeof(conf->x.pib_ind.txgain) + sizeof(conf->x.pib_ind.rxgain) +
                  sizeof(conf->x.pib_ind.sampling_rate) +
                  sizeof(conf->x.pib_ind.bandwidth) + sizeof(conf->x.pib_ind.signalstrength);
          dout << "PHY_msg_handler: indication erstellt"<<std::endl;
      }else{
          conf->type = PLME_SET_PHY_PIB_CONFIRM;
          conf->x.pib_conf.status = phy_SUCCESS;
          conf->x.pib_conf.cCAModeAttr = phy_msg_handler_impl::getCCAModeAttr();
          conf->x.pib_conf.chanSupAttr = 0;
          conf->x.pib_conf.currChanAttr = phy_msg_handler_impl::getCurrChanAttr();
          conf->x.pib_conf.CurrPageAttr = phy_msg_handler_impl::getCurrPageAttr();
          conf->x.pib_conf.LQI = phy_msg_handler_impl::getLQI();
          conf->x.pib_conf.maxFrDurAttr = phy_msg_handler_impl::getMaxFrDurAttr();
          conf->x.pib_conf.shdrDurAttr = phy_msg_handler_impl::getShdrDurAttr();
          conf->x.pib_conf.symOcAttr = phy_msg_handler_impl::getSymOcAttr();
          conf->x.pib_conf.trPwrAttr = phy_msg_handler_impl::getTrPwrAttr();
          conf->x.pib_conf.rxgain = phy_msg_handler_impl::getRxgain();
          conf->x.pib_conf.txgain = phy_msg_handler_impl::getTxgain();
          conf->x.pib_conf.bandwidth = phy_msg_handler_impl::getBandwidth();
          conf->x.pib_conf.sampling_rate = phy_msg_handler_impl::getSampling_rate();
          conf->x.pib_conf.signalstrength = phy_msg_handler_impl::getSignalstrength();
          conf->length =
                  sizeof(conf->type) + sizeof(conf->x.pib_conf.cCAModeAttr) + sizeof(conf->x.pib_conf.chanSupAttr)
                  + sizeof(conf->x.pib_conf.currChanAttr) + sizeof(conf->x.pib_conf.CurrPageAttr) +
                  sizeof(conf->x.pib_conf.LQI)
                  + sizeof(conf->x.pib_conf.maxFrDurAttr) + sizeof(conf->x.pib_conf.shdrDurAttr) +
                  sizeof(conf->x.pib_conf.symOcAttr)
                  + sizeof(conf->x.pib_conf.trPwrAttr) + sizeof(conf->x.pib_conf.status) +
                  sizeof(conf->x.pib_conf.txgain) + sizeof(conf->x.pib_conf.rxgain) +
                  sizeof(conf->x.pib_conf.sampling_rate) +
                  sizeof(conf->x.pib_conf.bandwidth) + sizeof(conf->x.pib_conf.signalstrength);
      }
    }

    int
    phy_msg_handler_impl::deserialize_msg(uint8_t *stream, PHY_msg *msg)
    {
      uint8_t *data = stream;
      msg->type = static_cast<msg_type_sap>(*data++);
      msg->length=*data++;
      switch (msg->type) {
          case PLME_SET_PHY_PIB_REQUEST:
              msg->x.pib_req.cCAModeAttr = *data++;
              msg->x.pib_req.chanSupAttr|= (uint32_t) *data++;
              msg->x.pib_req.chanSupAttr|= (uint32_t) *data++ << 8;
              msg->x.pib_req.chanSupAttr|= (uint32_t) *data++ << 16;
              msg->x.pib_req.chanSupAttr|= (uint32_t) *data++ << 24;
              msg->x.pib_req.currChanAttr = *data++;
              msg->x.pib_req.CurrPageAttr= *data++;
              msg->x.pib_req.LQI = *data++;
              msg->x.pib_req.maxFrDurAttr |= (uint16_t) *data++;
              msg->x.pib_req.maxFrDurAttr |= (uint16_t) *data++ << 8;
              msg->x.pib_req.shdrDurAttr = *data++;
              msg->x.pib_req.symOcAttr = *data++;
              msg->x.pib_req.trPwrAttr = *data++;
              msg->x.pib_req.rxgain = *data++;
              msg->x.pib_req.txgain = *data++;
              msg->x.pib_req.bandwidth |= (uint32_t) *data++;
              msg->x.pib_req.bandwidth |= (uint32_t) *data++ << 8;
              msg->x.pib_req.bandwidth |=(uint32_t)  *data++ << 16;
              msg->x.pib_req.bandwidth |=(uint32_t)  *data++ << 24;
              msg->x.pib_req.sampling_rate |= (uint32_t) *data++;
              msg->x.pib_req.sampling_rate |= (uint32_t) *data++ << 8;
              msg->x.pib_req.sampling_rate |= (uint32_t) *data++ << 16;
              msg->x.pib_req.sampling_rate |= (uint32_t) *data++ << 24;
              msg->x.pib_req.signalstrength = *data++;
              break;
          case PD_DATA_REQUEST:
              msg->x.data_req.psduLength = *data++;
              memcpy(&msg->x.data_req.data, data, msg->x.data_req.psduLength);
              data += msg->x.data_req.psduLength;
              break;
          case PD_DATA_CONFIRM:
              msg->x.data_conf.status = static_cast<phy_state>(*data++);
              break;
          case PD_DATA_INDICATION:
              msg->x.data_ind.psduLength = *data++;
              msg->x.data_ind.ppduLinkQuality = *data++;
              memcpy(&msg->x.data_ind.data, data, msg->x.data_ind.psduLength);
              data += msg->x.data_ind.psduLength;
              break;
          case PLME_CCA_REQUEST:
              break;
          case PLME_CCA_CONFIRM:
              msg->x.cca_conf.status = static_cast<phy_state>(*data++);
              break;
          case PLME_ED_REQUEST:
              break;
          case PLME_ED_CONFIRM:
              msg->x.ed_conf.status = static_cast<phy_state>(*data++);
              msg->x.ed_conf.energyLevel = *data++;
              break;
          case PLME_GET_REQUEST:
              msg->x.get_req.attribute = static_cast<phy_pib_attr>(*data++);
              break;
          case PLME_GET_CONFIRM:
              msg->x.get_conf.status = static_cast<phy_state>(*data++);
              msg->x.get_conf.attribute = static_cast<phy_pib_attr>(*data++);
              switch (msg->x.get_conf.attribute) {
                  case phyCurrentChannel:
                      msg->x.get_conf.value.currentChannel = *data++;
                      break;
                  case phyChannelsSupported:
                      msg->x.get_conf.value.channelsSupported |= (uint32_t) *data++;
                      msg->x.get_conf.value.channelsSupported |= (uint32_t) *data++ << 8;
                      msg->x.get_conf.value.channelsSupported |= (uint32_t) *data++ << 16;
                      msg->x.get_conf.value.channelsSupported |= (uint32_t) *data++ << 24;
                      break;
                  case phyTransmitPower:
                      msg->x.get_conf.value.transmitPower = *data++;
                      break;
                  case phyCCAMode:
                      msg->x.get_conf.value.cCAMode = *data++;
                      break;
                  case phyCurrentPage:
                      msg->x.get_conf.value.currentPage = *data++;
                      break;
                  case phyMaxFrameDuration:
                      msg->x.get_conf.value.maxFrameDuration |= (uint16_t) *data++;
                      msg->x.get_conf.value.maxFrameDuration |= (uint16_t) *data++ << 8;
                      break;
                  case phySHRDuration:
                      msg->x.get_conf.value.sHRDuration = *data++;
                      break;
                  case phySymbolsPerOctet:
                      msg->x.get_conf.value.symbolsPerOctet = *data++;
                      break;
                  default:
                      return -1;
              }
              break;
          case PLME_SET_TRX_STATE_REQUEST:
              msg->x.set_trx_state_req.status = static_cast<phy_state>(*data++);
              break;
          case PLME_SET_TRX_STATE_CONFIRM:
              msg->x.set_trx_state_conf.status = static_cast<phy_state>(*data++);
              break;
          case PLME_SET_REQUEST:
              msg->x.set_req.attribute = static_cast<phy_pib_attr>(*data++);
              switch (msg->x.set_req.attribute) {
                  case phyCurrentChannel:
                      msg->x.set_req.value.currentChannel=*data++;
                      break;
                  case phyChannelsSupported:
                      msg->x.set_req.value.channelsSupported |= (uint32_t) *data++;
                      msg->x.set_req.value.channelsSupported |= (uint32_t) *data++ << 8;
                      msg->x.set_req.value.channelsSupported |= (uint32_t) *data++ << 16;
                      msg->x.set_req.value.channelsSupported |= (uint32_t) *data++ << 24;
                      break;
                  case phyTransmitPower:
                      msg->x.set_req.value.transmitPower = *data++;
                      break;
                  case phyCCAMode:
                      msg->x.set_req.value.cCAMode = *data++;
                      break;
                  case phyCurrentPage:
                      msg->x.set_req.value.currentPage = *data++;
                      break;
                  case phyMaxFrameDuration:
                      msg->x.set_req.value.maxFrameDuration |= (uint16_t) *data++;
                      msg->x.set_req.value.maxFrameDuration |= (uint16_t) *data++ << 8;
                      break;
                  case phySHRDuration:
                      msg->x.set_req.value.sHRDuration = *data++;
                      break;
                  case phySymbolsPerOctet:
                      msg->x.set_req.value.symbolsPerOctet = *data++;
                      break;
                  case phyRxGain:
                      msg->x.set_req.value.RxGain = *data++;
                      break;
                  case phyTxGain:
                      msg->x.set_req.value.TxGain = *data++;
                      break;
                  case phyBandwidth:
                      msg->x.set_req.value.Bandwidth |= (uint32_t) *data++;
                      msg->x.set_req.value.Bandwidth |= (uint32_t) *data++ << 8;
                      msg->x.set_req.value.Bandwidth |= (uint32_t) *data++ << 16;
                      msg->x.set_req.value.Bandwidth |= (uint32_t) *data++ << 24;
                      break;
                  case phySamplingRate:
                      msg->x.set_req.value.SamplingRate |= (uint32_t) *data++;
                      msg->x.set_req.value.SamplingRate |= (uint32_t) *data++ << 8;
                      msg->x.set_req.value.SamplingRate |= (uint32_t) *data++ << 16;
                      msg->x.set_req.value.SamplingRate |= (uint32_t) *data++ << 24;
                      break;
                  default:
                      return -1;
              }
              break;
          case PLME_SET_CONFIRM:
              break;
          default:
              return -1;
      }

      return (int) (data - stream);
    }

    int
    phy_msg_handler_impl::serializes_msg(PHY_msg * msg, uint8_t buffer[aMaxPHYPacketSize])
    {
      uint8_t pos = 0;

      buffer[pos++] = msg->type;
      buffer[pos++] = msg->length;

      switch (msg->type) {
          case PLME_SET_PHY_PIB_INDICATION:{
              buffer[pos++] = msg->x.pib_ind.status;
              buffer[pos++] = msg->x.pib_ind.cCAModeAttr;
              buffer[pos++] = (uint8_t) msg->x.pib_ind.chanSupAttr;
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.chanSupAttr >> 8);
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.chanSupAttr >> 16);
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.chanSupAttr >> 24);
              buffer[pos++] = msg->x.pib_ind.currChanAttr;
              buffer[pos++] = msg->x.pib_ind.CurrPageAttr;
              buffer[pos++] = msg->x.pib_ind.LQI;
              buffer[pos++] = (uint8_t) msg->x.pib_ind.maxFrDurAttr;
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.maxFrDurAttr >> 8);
              buffer[pos++] = msg->x.pib_ind.shdrDurAttr;
              buffer[pos++] = msg->x.pib_ind.symOcAttr;
              buffer[pos++] = msg->x.pib_ind.trPwrAttr;
              buffer[pos++] = msg->x.pib_ind.rxgain;
              buffer[pos++] = msg->x.pib_ind.txgain;
              buffer[pos++] = (uint8_t) msg->x.pib_ind.bandwidth;
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.bandwidth >> 8);
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.bandwidth >> 16);
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.bandwidth >> 24);
              buffer[pos++] = (uint8_t) msg->x.pib_ind.sampling_rate;
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.sampling_rate >> 8);
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.sampling_rate >> 16);
              buffer[pos++] = (uint8_t) (msg->x.pib_ind.sampling_rate >> 24);
              buffer[pos++] = msg->x.pib_ind.signalstrength;
              break;
          }
          case PLME_SET_PHY_PIB_CONFIRM:
              buffer[pos++]=msg->x.pib_conf.status;
              buffer[pos++]=msg->x.pib_conf.cCAModeAttr;
              buffer[pos++] = (uint8_t) msg->x.pib_conf.chanSupAttr;
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.chanSupAttr >> 8);
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.chanSupAttr >> 16);
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.chanSupAttr >> 24);
              buffer[pos++]=msg->x.pib_conf.currChanAttr;
              buffer[pos++]=msg->x.pib_conf.CurrPageAttr;
              buffer[pos++]=msg->x.pib_conf.LQI;
              buffer[pos++] = (uint8_t) msg->x.pib_conf.maxFrDurAttr;
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.maxFrDurAttr >> 8);
              buffer[pos++]=msg->x.pib_conf.shdrDurAttr;
              buffer[pos++]=msg->x.pib_conf.symOcAttr;
              buffer[pos++]=msg->x.pib_conf.trPwrAttr;
              buffer[pos++]=msg->x.pib_conf.rxgain;
              buffer[pos++]=msg->x.pib_conf.txgain;
              buffer[pos++] = (uint8_t) msg->x.pib_conf.bandwidth;
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.bandwidth >> 8);
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.bandwidth >> 16);
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.bandwidth >> 24);
              buffer[pos++] = (uint8_t) msg->x.pib_conf.sampling_rate;
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.sampling_rate >> 8);
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.sampling_rate >> 16);
              buffer[pos++] = (uint8_t) (msg->x.pib_conf.sampling_rate >> 24);
              buffer[pos++]=msg->x.pib_conf.signalstrength;
              break;
          case PD_DATA_REQUEST:
              buffer[pos++] = msg->x.data_req.psduLength;
              memcpy(&buffer[pos], (void *)msg->x.data_req.data, msg->x.data_req.psduLength);
              pos += msg->x.data_req.psduLength;
              break;
          case PD_DATA_CONFIRM:
              buffer[pos++] = msg->x.data_conf.status;
              break;
          case PD_DATA_INDICATION:
              buffer[pos++] = msg->x.data_ind.psduLength;
              buffer[pos++] = msg->x.data_ind.ppduLinkQuality;
              memcpy(&buffer[pos], (void *)msg->x.data_ind.data, msg->x.data_ind.psduLength);
              pos += msg->x.data_ind.psduLength;
              break;
          case PLME_CCA_REQUEST:
              break;
          case PLME_CCA_CONFIRM:
              buffer[pos++] = msg->x.cca_conf.status;
              break;
          case PLME_ED_REQUEST:
              break;
          case PLME_ED_CONFIRM:
              buffer[pos++] = msg->x.ed_conf.status;
              buffer[pos++] = msg->x.ed_conf.energyLevel;
              break;
          case PLME_GET_REQUEST:
              buffer[pos++] = msg->x.get_req.attribute;
              break;
          case PLME_GET_CONFIRM:
              buffer[pos++] = msg->x.get_conf.status;
              buffer[pos++] = msg->x.get_conf.attribute;
              switch (msg->x.get_conf.attribute) {
                  case phyCurrentChannel:
                      buffer[pos++] = msg->x.get_conf.value.currentChannel;
                      break;
                  case phyChannelsSupported:
                      buffer[pos++] = (uint8_t) msg->x.get_conf.value.channelsSupported;
                      buffer[pos++] = (uint8_t) (msg->x.get_conf.value.channelsSupported >> 8);
                      buffer[pos++] = (uint8_t) (msg->x.get_conf.value.channelsSupported >> 16);
                      buffer[pos++] = (uint8_t) (msg->x.get_conf.value.channelsSupported >> 24);
                      break;
                  case phyTransmitPower:
                      buffer[pos++] = msg->x.get_conf.value.transmitPower;
                      break;
                  case phyCCAMode:
                      buffer[pos++] = msg->x.get_conf.value.cCAMode;
                      break;
                  case phyCurrentPage:
                      buffer[pos++] = msg->x.get_conf.value.currentPage;
                      break;
                  case phyMaxFrameDuration:
                      buffer[pos++] = (uint8_t) msg->x.get_conf.value.maxFrameDuration;
                      buffer[pos++] = (uint8_t) (msg->x.get_conf.value.maxFrameDuration >> 8);
                      break;
                  case phySHRDuration:
                      buffer[pos++] = msg->x.get_conf.value.sHRDuration;
                      break;
                  case phySymbolsPerOctet:
                      buffer[pos++] = msg->x.get_conf.value.symbolsPerOctet;
                      break;
                  default:
                      return -1;
              }
              break;
          case PLME_SET_TRX_STATE_REQUEST:
              buffer[pos++] = msg->x.set_trx_state_req.status;
              break;
          case PLME_SET_TRX_STATE_CONFIRM:
              buffer[pos++] = msg->x.set_trx_state_conf.status;
              break;
          case PLME_SET_REQUEST:
              buffer[pos++] = msg->x.set_req.attribute;
              switch (msg->x.set_req.attribute) {
                  case phyCurrentChannel:
                      buffer[pos++] = msg->x.set_req.value.currentChannel;
                      break;
                  case phyChannelsSupported:
                      buffer[pos++] = (uint8_t) msg->x.set_req.value.channelsSupported;
                      buffer[pos++] = (uint8_t) (msg->x.set_req.value.channelsSupported >> 8);
                      buffer[pos++] = (uint8_t) (msg->x.set_req.value.channelsSupported >> 16);
                      buffer[pos++] = (uint8_t) (msg->x.set_req.value.channelsSupported >> 24);
                      break;
                  case phyTransmitPower:
                      buffer[pos++] = msg->x.set_req.value.transmitPower;
                      break;
                  case phyCCAMode:
                      buffer[pos++] = msg->x.set_req.value.cCAMode;
                      break;
                  case phyCurrentPage:
                      buffer[pos++] = msg->x.set_req.value.currentPage;
                      break;
                  case phyMaxFrameDuration:
                      buffer[pos++] = (uint8_t) msg->x.set_req.value.maxFrameDuration;
                      buffer[pos++] = (uint8_t) (msg->x.set_req.value.maxFrameDuration >> 8);
                      break;
                  case phySHRDuration:
                      buffer[pos++] = msg->x.set_req.value.sHRDuration;
                      break;
                  case phySymbolsPerOctet:
                      buffer[pos++] = msg->x.set_req.value.symbolsPerOctet;
                      break;
                  default:
                      return -1;
              }
              break;
          case PLME_SET_CONFIRM:
              buffer[pos++] = msg->x.set_conf.status;
              buffer[pos++] = msg->x.set_conf.attribute;
              break;
          default:
              return -1;
      }

      return pos;

    }

    uint8_t
    phy_msg_handler_impl::getCurrChanAttr() const
    {
        dout << "PHY_msg_handler: getCurrChanAttr():" << currChanAttr << std::endl;
      return currChanAttr;
    }

    void
    phy_msg_handler_impl::setCurrChanAttr(uint8_t currChanAttr)
    {
        dout << "PHY_msg_handler: setCurrChanAttr():" << currChanAttr << std::endl;
      phy_msg_handler_impl::currChanAttr = currChanAttr;
      short k=currChanAttr;
      uint64_t freq=0;
      if (k == 0) {
          freq = 8683;
      }
      if (k > 1 && k <= 10) {
          freq = 906 + 2 * (k - 1);
      }
      if (k > 10) {
          freq = 2405 + 5 * (k - 11);
      }
      freq = freq * 1000000;

      pmt::pmt_t CMD_FREQ_KEY= pmt::mp("freq");
      pmt::pmt_t dict= pmt::make_dict();
      dict=pmt::dict_add(dict,pmt::intern("freq"),pmt::from_double(freq));
        dout << "PHY_msg_handler: freq in phy: "<<pmt::from_double(freq)<<sizeof(double)<<std::endl;
      message_port_pub(pmt::mp("manag_tx_out"), dict);
      message_port_pub(pmt::mp("manag_rx_out"), dict);
      if (phy_msg_handler_impl::getCenter_freq() != freq){
          phy_msg_handler_impl::setCenter_freq(freq);
      }

    }

    uint32_t
    phy_msg_handler_impl::getChanSupAttr() const
    {
        dout << "PHY_msg_handler: getChanSupAttr(): " << this->chanSupAttr << std::endl;
      return chanSupAttr;
    }

    void
    phy_msg_handler_impl::setChanSupAttr(uint32_t chanSupAttr)
    {
        dout << "PHY_msg_handler: setChanSupAttr(): " << chanSupAttr << std::endl;
      phy_msg_handler_impl::chanSupAttr = chanSupAttr;
    }

    uint8_t
    phy_msg_handler_impl::getTrPwrAttr() const
    {
        dout << "PHY_msg_handler: getTrPwrAttr(): " << this->trPwrAttr << std::endl;
      return trPwrAttr;
    }

    void
    phy_msg_handler_impl::setTrPwrAttr(uint8_t trPwrAttr)
    {
        dout << "PHY_msg_handler: setTrPwrAttr(): " << trPwrAttr << std::endl;
      phy_msg_handler_impl::trPwrAttr = trPwrAttr;
    }

    uint8_t
    phy_msg_handler_impl::getCCAModeAttr() const
    {
        dout << "PHY_msg_handler: getCCAModeAttr(): " << this->cCAModeAttr << std::endl;
      return cCAModeAttr;
    }

    void
    phy_msg_handler_impl::setCCAModeAttr(uint8_t cCAModeAttr)
    {
        dout << "PHY_msg_handler: setCCAModeAttr(): " << cCAModeAttr << std::endl;
      phy_msg_handler_impl::cCAModeAttr = cCAModeAttr;
    }

    uint8_t
    phy_msg_handler_impl::getCurrPageAttr() const
    {
        dout << "PHY_msg_handler: getCurrPageAttr(): " << this->CurrPageAttr << std::endl;
      return CurrPageAttr;
    }

    void
    phy_msg_handler_impl::setCurrPageAttr(uint8_t CurrPageAttr)
    {
        dout << "PHY_msg_handler: setCurrPageAttr(): " << CurrPageAttr << std::endl;
      phy_msg_handler_impl::CurrPageAttr = CurrPageAttr;
    }

    uint16_t
    phy_msg_handler_impl::getMaxFrDurAttr() const
    {
        dout << "PHY_msg_handler: getMaxFrDurAttr(): " << this->maxFrDurAttr << std::endl;
      return maxFrDurAttr;
    }

    void
    phy_msg_handler_impl::setMaxFrDurAttr(uint16_t maxFrDurAttr)
    {
        dout << "PHY_msg_handler: setMaxFrDurAttr(): " << maxFrDurAttr << std::endl;
      phy_msg_handler_impl::maxFrDurAttr = maxFrDurAttr;
    }

    uint8_t
    phy_msg_handler_impl::getShdrDurAttr() const
    {
        dout << "PHY_msg_handler: getShdrDurAttr(): " << this->shdrDurAttr << std::endl;
      return shdrDurAttr;
    }

    void
    phy_msg_handler_impl::setShdrDurAttr(uint8_t shdrDurAttr)
    {
        dout << "PHY_msg_handler: setShdrDurAttr(): " << shdrDurAttr << std::endl;
      phy_msg_handler_impl::shdrDurAttr = shdrDurAttr;
    }

    uint8_t
    phy_msg_handler_impl::getSymOcAttr() const
    {
        dout << "PHY_msg_handler: getSymOcAttr(): " << this->symOcAttr << std::endl;
      return symOcAttr;
    }

    void
    phy_msg_handler_impl::setSymOcAttr(uint8_t symOcAttr)
    {
        dout << "PHY_msg_handler: setSymOcAttr(): " << symOcAttr << std::endl;
      phy_msg_handler_impl::symOcAttr = symOcAttr;
    }

    uint8_t
    phy_msg_handler_impl::getLQI() const
    {
        dout << "PHY_msg_handler: getLQI(): " << this->LQI << std::endl;
      return LQI;
    }

    void
    phy_msg_handler_impl::setLQI(uint8_t LQI)
    {
        dout << "PHY_msg_handler: setLQI(): " << LQI << std::endl;
      phy_msg_handler_impl::LQI = LQI;
    }

    uint8_t
    phy_msg_handler_impl::getTxgain() const
    {
        dout << "PHY_msg_handler: getTxgain(): " << this->txgain << std::endl;
      return txgain;
    }

    void
    phy_msg_handler_impl::setTxgain(uint8_t txgain)
    {
        dout << "PHY_msg_handler: setTxgain(): " << txgain << std::endl;
      pmt::pmt_t dict= pmt::make_dict();
      dict=pmt::dict_add(dict,pmt::intern("gain"),pmt::from_double((double)txgain));//Bei gain=60 wird der Sendegain genutzt
      message_port_pub(pmt::mp("manag_tx_out"), dict);
      phy_msg_handler_impl::txgain = txgain;
    }

    uint8_t
    phy_msg_handler_impl::getRxgain() const
    {
        dout << "PHY_msg_handler: getRxgain(): " << this->rxgain << std::endl;
      return rxgain;
    }

    void
    phy_msg_handler_impl::setRxgain(uint8_t rxgain)
    {
        dout << "PHY_msg_handler: setRxgain(): " << rxgain << std::endl;
      pmt::pmt_t dict= pmt::make_dict();
      dict=pmt::dict_add(dict,pmt::intern("gain"),pmt::from_double((double)rxgain));//Bei gain=14 wird der Empfangsgain genutzt
      message_port_pub(pmt::mp("manag_rx_out"), dict);
      phy_msg_handler_impl::rxgain = rxgain;
    }

    uint32_t
    phy_msg_handler_impl::getBandwidth() const
    {
        dout << "PHY_msg_handler: getBandwidth(): " << this->bandwidth << std::endl;
      return bandwidth;
    }

    void
    phy_msg_handler_impl::setBandwidth(uint32_t bandwidth)
    {
        dout << "PHY_msg_handler: setBandwidth(): " << bandwidth << std::endl;
      pmt::pmt_t dict= pmt::make_dict();
      dict=pmt::dict_add(dict,pmt::intern("bw"),pmt::from_double((double)bandwidth));
      message_port_pub(pmt::mp("manag_tx_out"), dict);
      phy_msg_handler_impl::bandwidth = bandwidth;
    }

    uint32_t
    phy_msg_handler_impl::getSampling_rate() const
    {
        dout << "PHY_msg_handler: getSampling_rate(): " << this->sampling_rate << std::endl;
      return sampling_rate;
    }

    void
    phy_msg_handler_impl::setSampling_rate(uint32_t sampling_rate)
    {
        dout << "PHY_msg_handler: setSampling_rate(): " << sampling_rate << std::endl;
      pmt::pmt_t dict= pmt::make_dict();
      dict=pmt::dict_add(dict,pmt::intern("samp_rate"),pmt::from_double((double)sampling_rate));
      message_port_pub(pmt::mp("manag_tx_out"), dict);
      phy_msg_handler_impl::sampling_rate = sampling_rate;
    }

    uint64_t
    phy_msg_handler_impl::getCenter_freq() const
    {
        dout << "PHY_msg_handler: getCenter_freq(): " << this->center_freq << std::endl;
      return center_freq;
    }

    void
    phy_msg_handler_impl::setCenter_freq(uint64_t center_freq)
    {
        dout << "PHY_msg_handler: setCenter_freq(): " << center_freq << std::endl;
      phy_msg_handler_impl::center_freq = center_freq;
      uint8_t chan;
      chan=static_cast<uint8_t >(((center_freq/1000000)/5)-470);

      if (phy_msg_handler_impl::getCurrChanAttr()!= chan){
          phy_msg_handler_impl::setCurrChanAttr(chan);
      }
    }

    double
    phy_msg_handler_impl::getSignalstrength() const
    {
        dout << "PHY_msg_handler: getSignalStrength(): " << this->signalstrength << std::endl;
      return signalstrength;
    }

    void
    phy_msg_handler_impl::setSignalstrength(double signalstrength)
    {
        //dout << "PHY_msg_handler: setSignalStrength(): " << signalstrength << std::endl;
      phy_msg_handler_impl::signalstrength = signalstrength;
    }

    uint8_t
    phy_msg_handler_impl::getPibsignalstr() const
    {
        dout << "PHY_msg_handler: getPibsignalstr(): " << this->pibsignalstr << std::endl;
      return pibsignalstr;
    }

    void
    phy_msg_handler_impl::setPibsignalstr(double signal)
    {
        dout << "PHY_msg_handler: setPibsignalstr(): " << signal << std::endl;
    //      Messwert wird über lineare Funktion auf Wertebereich 0 ... 255 angepasst.
      uint8_t ss;
      ss=(uint8_t)((3.05*signal)+260); //eventuell anpassen!
      ss= ss < 255 ? ss: 255; // maximal wert begrenzen -> da unsigned kleiner 0 nicht möglich
      phy_msg_handler_impl::pibsignalstr = ss;
    }

    int
    phy_msg_handler_impl::getCca_threshold() const
    {
        dout << "PHY_msg_handler: getCca_treshold(): " << this->cca_threshold << std::endl;
      return cca_threshold;
    }

    void
    phy_msg_handler_impl::setCca_threshold(int cca_threshold)
    {
        dout << "PHY_msg_handler: setCca_treshold(): " << cca_threshold << std::endl;
      phy_msg_handler_impl::cca_threshold = cca_threshold;
    }

    void
    phy_msg_handler_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    phy_msg_handler_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        throw std::runtime_error("not a stream block");
    }

  } /* namespace ieee802_15_4 */
} /* namespace gr */

