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

#ifndef INCLUDED_IEEE802_15_4_PHY_MSG_HANDLER_IMPL_H
#define INCLUDED_IEEE802_15_4_PHY_MSG_HANDLER_IMPL_H

#include <ieee802_15_4/phy_msg_handler.h>
#include "include/serial-phy.h"

namespace gr {
  namespace ieee802_15_4 {

    class phy_msg_handler_impl : public phy_msg_handler
    {
    private:
        uint8_t packet_couner=0;

        void handle_pdu(pmt::pmt_t pdu);
        void handle_phymsg(pmt::pmt_t pdu);
        void handle_signalmsg(pmt::pmt_t pdu);
        int deserialize_msg(uint8_t *stream,PHY_msg *msg);
        int serializes_msg(PHY_msg * msg, uint8_t buffer[aMaxPHYPacketSize]);
        void handle_msg(PHY_msg * msg);
        void send_conf(PHY_msg conf);
        void get_PIB_attrs(PHY_msg* conf,bool ind);
        void performED();
        void performCCA();
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
        uint64_t center_freq;
        double signalstrength=-90.0;
        uint8_t pibsignalstr=0;
        short signalstrenghtcounter;
        bool ed;
        bool cca;
        bool start;
        int cca_threshold;

        bool d_debug;

     public:
        phy_msg_handler_impl(uint64_t center_freq0,uint32_t samp_rate, uint32_t bw0, uint8_t txg, uint8_t rxg, bool debug);
        ~phy_msg_handler_impl();

      // Where all the action really happens
      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

	//bool start();

        uint8_t getCurrChanAttr() const;
        void setCurrChanAttr(uint8_t currChanAttr);

        uint32_t getChanSupAttr() const;
        void setChanSupAttr(uint32_t chanSupAttr);

        uint8_t getTrPwrAttr() const;
        void setTrPwrAttr(uint8_t trPwrAttr);

        uint8_t getCCAModeAttr() const;
        void setCCAModeAttr(uint8_t cCAModeAttr);

        uint8_t getCurrPageAttr() const;
        void setCurrPageAttr(uint8_t CurrPageAttr);

        uint16_t getMaxFrDurAttr() const;
        void setMaxFrDurAttr(uint16_t maxFrDurAttr);

        uint8_t getShdrDurAttr() const;
        void setShdrDurAttr(uint8_t shdrDurAttr);

        uint8_t getSymOcAttr() const;
        void setSymOcAttr(uint8_t symOcAttr);

        phy_state getTRXstatus() const;
        void setTRXstatus(phy_state TRXstatus);

        uint8_t getLQI() const;
        void setLQI(uint8_t LQI);

        uint8_t getTxgain() const;
        void setTxgain(uint8_t txgain);

        uint8_t getRxgain() const;
        void setRxgain(uint8_t rxgain);

        uint32_t getBandwidth() const;
        void setBandwidth(uint32_t bandwidth);

        uint32_t getSampling_rate() const;
        void setSampling_rate(uint32_t sampling_rate);

        uint64_t getCenter_freq() const;
        void setCenter_freq(uint64_t center_freq);

        uint8_t getPibsignalstr() const;
        void setPibsignalstr(double signal);

        int getCca_threshold() const;
        void setCca_threshold(int cca_threshold);

        double getSignalstrength() const;
        void setSignalstrength(double signalstrength);

      };

  } // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_PHY_MSG_HANDLER_IMPL_H */

