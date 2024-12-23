#ifndef EBYTE_E32
#define EBYTE_E32

// Head settings
enum Head {
    SAVE_ON_EXIT = 0xC0,
    NO_SAVE_ON_EXIT = 0xC2
};

// UART parity options
enum UartParity {
    NO_PARITY,
    ODD_PARITY,
    EVEN_PARITY
};

// UART baud rate options
enum UartBaudRate {
    BAUD_1200,
    BAUD_2400,
    BAUD_4800,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200
};

// Air data rate options
enum AirRate {
    AIR_RATE_2400,
    AIR_RATE_2400_1,
    AIR_RATE_2400_2,
    AIR_RATE_4800,
    AIR_RATE_9600,
    AIR_RATE_19200,
    AIR_RATE_19200_1,
    AIR_RATE_19200_2
};

// Communication channels
enum CommunicationChannel {
    CHANNEL_0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    CHANNEL_9,
    CHANNEL_10,
    CHANNEL_11,
    CHANNEL_12,
    CHANNEL_13,
    CHANNEL_14,
    CHANNEL_15,
    CHANNEL_16,
    CHANNEL_17,
    CHANNEL_18,
    CHANNEL_19,
    CHANNEL_20,
    CHANNEL_21,
    CHANNEL_22,
    CHANNEL_23,
    CHANNEL_24,
    CHANNEL_25,
    CHANNEL_26,
    CHANNEL_27,
    CHANNEL_28,
    CHANNEL_29,
    CHANNEL_30,
    CHANNEL_31
};

// Transparency modes
enum TransparencyMode {
    TRANSPARENT,
    FIXED
};

// IO drive modes
enum IoDrive {
    OPEN_COLLECTOR,
    PUSH_PULL
};

// Wake-up time options
enum WakeUpTime {
    WAKE_250MS,
    WAKE_500MS,
    WAKE_750MS,
    WAKE_1000MS,
    WAKE_1250MS,
    WAKE_1500MS,
    WAKE_1750MS,
    WAKE_2000MS
};

// Forward Error Correction options
enum ForwardErrorCorrection {
    FEC_OFF,
    FEC_ON
};

// Transmission power levels
enum TransmissionPower {
    POWER_30DBM,
    POWER_27DBM,
    POWER_24DBM,
    POWER_21DBM
};

// Struct for Ebyte module settings
struct EbyteSettings {
    Head head;
    UartParity parity;
    UartBaudRate baud;
    AirRate air_rate;
    CommunicationChannel channel;
    TransparencyMode mode;
    IoDrive drive;
    WakeUpTime wake;
    ForwardErrorCorrection fec;
    TransmissionPower power;

    // Overloading the equality operator
    bool operator==(const EbyteSettings& other) const {
        return head == other.head &&
               parity == other.parity &&
               baud == other.baud &&
               air_rate == other.air_rate &&
               channel == other.channel &&
               mode == other.mode &&
               drive == other.drive &&
               wake == other.wake &&
               fec == other.fec &&
               power == other.power;
    }
};

#endif