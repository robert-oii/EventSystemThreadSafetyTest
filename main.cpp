// Thread sanitizer seems to be broken on some versions of gcc.
// For example on Ubuntu Desktop 22.04, gcc 11.4.0's thread sanitizer is broken.
// Possible fixes are given here:
//   https://github.com/google/sanitizers/issues/1716
// In the meantime just compile without thread sanitizer and use helgrind to detect
// any data races.
//
//   sudo apt update
//   sudo apt install -y valgrind
//   mkdir ~/EventsThreadTest/build && cd $_
//   cmake .. -DSANITIZERS=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
//   cmake --build .
//   valgrind --tool=helgrind ./events_thread_test

#include <string>
#include <cstdint>
#include <cstring> // memcpy
#include <vector>
#include <cstdio>
#include <unordered_map>
#include <cassert>

using Event = uint16_t;

const int MAXEVENTS = 65500;

void sysErrorEx( const char* trigstr, const char* comment, const char* file, int line ) {}

#define sysFatalError( comment ) sysErrorEx( "True", (comment), __FILE__, __LINE__ )
#define sysError( trigger, comment ) ((trigger) ? sysErrorEx( #trigger, (comment), __FILE__, __LINE__ ) : (void)NULL)

// Events
enum eventType
{
    unknownType = 0,
    legacyType = 1,
    doubleType,
    blockUInt32Type,
    block32Type,
    block64Type,
    eventTypeMask = 0x7,
    legacyTypeBool = 0x11,
    legacyTypeShort = 0x21,
    legacyTypeLong = 0x31
};

typedef union
{
    uint8_t* blockPtr;
    double* doublePtr;
    int32_t		i32Val;
} lwb;

// Accepter functions return true if the value was accepted as is (without being changed).  The return value had to be
// added to fix the following scenario.  A remote client modifies a variable to 101.  The variable has an accepter function
// attached that clips the value between -100 and 100.  The current value of the variable is 100.  When the value 101 shows
// up in the sync logic, the value is set to 101 but the accepter keeps it at 100.  This means the variable didn't change
// its value, so the variable did not get stimulated.  Now the remote client has the value set to 101 and CTAG has the
// value set to 100 (mismatch).  We need the accepter to tell us if the value was modified so that the variable can be
// stimulated to keep all remote clients in sync.
union Accepter
{
    bool ( *mLegacyAccepter )( void* arg, Event e, int32_t* value );
    bool ( *mDoubleAccepter )( void* arg, Event e, double* value );
    bool ( *mBlockAccepter )( void* arg, Event e, const void** value, int16_t* num );
};


struct AccepterWithArg
{
    Accepter mAccepter;		// The type specific accepter functions.  The value is passed to the accepter function before being set in the variable.
    void* mArg;				// A void pointer argument to be passed to the accepter function.
};

struct EventData
{
    lwb mValue;									// The type specific value.
    eventType mType;							// The value type.
    std::vector<Event>* mLinks;					// List of event ids linked to this event (e.g. sLinks = (2, 6): This event is linked with events 2 and 6).
    std::vector<AccepterWithArg>* mAccepters;	// List of value accepter functions to call before setting the event.
};

static EventData* events;
static Event poolind;

void init_events( int16_t poolstart )
{
    poolind = poolstart;
    events = new EventData[ MAXEVENTS ];
}

void SetEventType( Event e, eventType type )
{
    sysError( e <= 0 || e >= MAXEVENTS, "Bad event id" );
    eventType currentType = events[ e ].mType;
    if ( currentType == unknownType || ( currentType == legacyType && type >= legacyTypeBool ) )
    {
        events[ e ].mType = type;
    }
    else if ( currentType != type )
    {
        char buffer[ 80 ];

        snprintf( buffer, 80, "Event type has already been set [event %d changing from %d to %d]", e, currentType, type );
        sysError( currentType != type, buffer );
    }
}

int16_t getblockevent( Event e, uint8_t* value, int16_t size )
{
    int16_t num = 0;
    if ( events[ e ].mValue.blockPtr != nullptr )
    {
        num = (int16_t)( events[ e ].mValue.blockPtr[ 0 ] );
        sysError( num < 0 || num > size, "Invalid block size" );
        memcpy( value, events[ e ].mValue.blockPtr + 1, num );
    }
    return num;
}

int16_t getb32event( Event e, uint8_t* value )
{
    SetEventType( e, block32Type );
    return getblockevent( e, value, 32 );
}

std::string GetAscii32( Event e )
{
    uint8_t buffer[ 33 ];
    int16_t num = getb32event( e, buffer );
    buffer[ num ] = '\0';
    return (char*)buffer;
}

int32_t getlevent( Event e )
{
    SetEventType( e, legacyType );
    return ( events[ e ].mValue.i32Val );
}

static int32_t getlegacy( Event e, eventType type )
{
    SetEventType( e, type );
    return ( events[ e ].mValue.i32Val );
}

#define NOVALUE         0x7FFFFFFFL
#define INOVALUE		0x7FFF

int16_t	GetS16( Event e )
{
    int32_t value = getlegacy( e, legacyTypeShort );
    return (int16_t)( value == NOVALUE ? INOVALUE : value );
}

int32_t	GetS32( Event e )
{
    SetEventType( e, legacyTypeLong );
    return ( events[ e ].mValue.i32Val );
}

void sysIntDisable() {}
void sysIntEnable() {}
void suspend() {}
void resume() {}

static void putlegacy( Event e, int32_t value, eventType type );

void putAsLegacyEvent( Event e, int32_t value )
{	// intended for core functions only, to help support new legacy sub-types
    if ( events[ e ].mType >= legacyTypeBool )
        putlegacy( e, value, events[ e ].mType );
    else
        putlegacy( e, value, legacyType );
}

void stimevent( Event e ) {}

static void putlegacy( Event e, int32_t value, eventType type )
{
    SetEventType( e, type );

    sysIntDisable();
    suspend();

    std::vector<AccepterWithArg>* accepters = events[ e ].mAccepters;
    if ( accepters != nullptr )
    {
        for ( std::vector<AccepterWithArg>::iterator it = accepters->begin(); it != accepters->end(); ++it )
        {
            if ( ( *it ).mAccepter.mLegacyAccepter( ( *it ).mArg, e, &value ) == false ) stimevent( e );
        }
    }

    if ( value != events[ e ].mValue.i32Val )
    {
#ifdef EVENT_DEBUG
        auto now = std::chrono::high_resolution_clock::now();
        uint32_t duration = static_cast<uint32_t>( std::chrono::duration_cast<std::chrono::milliseconds>( now - events[ e ].mLastCall ).count() );
        events[ e ].mLastCall = now;
        if ( duration < 5 && events[ STARTUP ].mValue.i32Val == 1 && events[ e ].mLinks )
            ++events[ e ].mCycleCount;
#endif
        // The value has changed, so update the variable and stimulate it's actions.
        events[ e ].mValue.i32Val = value;
        stimevent( e );	// Stimulate this event first so its actions get on the queue before the linked events.

        // Set all of the linked events to the same value.
        if ( events[ e ].mLinks != nullptr ) for ( uint32_t i = 0; i < events[ e ].mLinks->size(); ++i ) putAsLegacyEvent( events[ e ].mLinks->at( i ), value );
    }

    resume();
    sysIntEnable();
}

void inclevent( Event e )
{
    int32_t lval = 0;
    switch ( events[ e ].mType )
    {
        case unknownType:
        case legacyType:
            lval = getlevent( e );
            break;
        case legacyTypeShort:
            lval = GetS16( e );
            break;
        case legacyTypeLong:
            lval = GetS32( e );
            break;
        default:
        {
            char buffer[ 80 ];
            snprintf( buffer, 80, "inclevent(%d) called for wrong event type %x", e, events[ e ].mType );
            sysFatalError( buffer );
        }
        break;
    }
    putlegacy( e, lval + 1, events[ e ].mType );
}

constexpr Event NOEVENT = 0;

// Types
enum	// 16 bits
{
    WT_SINT32 = 2305,			// long
};

// Units
enum
{
    WT_UNITS_NONE = 0,
    WT_UNITS_TEMPER = 1,
    WT_UNITS_PRESSR = 2,
    WT_UNITS_LENGTH = 3,
    WT_UNITS_FLOW = 4,
    WT_UNITS_PASSWORD = 5,

    WT_UNITS_MASK = 0x0F,
};

// Type class (TCL)
enum	// 8 bits
{
    WT_TCL_BIT_POS = 8,
    WT_TCL_NUM_BITS = 8,
    WT_TCL_MASK = ( ( 1 << WT_TCL_NUM_BITS ) - 1 ) << WT_TCL_BIT_POS,

    // flag for new subtypes
    WT_NEW = 0x80 << WT_TCL_BIT_POS,
    RT_TCL_MASK = WT_NEW - 1,

    // size 0
    WT_TCL_SZ0_B1 = 0 << WT_TCL_BIT_POS,	// boolean
    RT_TCL_SZ0_B1 = WT_TCL_SZ0_B1 | WT_NEW,  // boolean, restricted type

    // size 1
    WT_TCL_SZ1_B8 = 1 << WT_TCL_BIT_POS,	// 8-bit unsigned integer
    WT_TCL_SZ1_UN = 2 << WT_TCL_BIT_POS,	// units specification
    WT_TCL_SZ1_EN = 3 << WT_TCL_BIT_POS,	// enum type
    RT_TCL_SZ1_B8 = WT_TCL_SZ1_B8 | WT_NEW,  // one-byte, restricted type
    RT_TCL_SZ1_UN = WT_TCL_SZ1_UN | WT_NEW,  // one-byte, restricted type
    RT_TCL_SZ1_EN = WT_TCL_SZ1_EN | WT_NEW,  // one-byte, restricted type

    // size 2
    WT_TCL_SZ2_B16 = 4 << WT_TCL_BIT_POS,	// 16 bit value
    WT_TCL_SZ2_B15 = 5 << WT_TCL_BIT_POS,	// 15 bit value
    WT_TCL_SZ2_B12 = 6 << WT_TCL_BIT_POS,	// 12 bit value
    WT_TCL_SZ2_B10 = 7 << WT_TCL_BIT_POS,	// 10 bit value
    WT_TCL_SZ2_STR = 8 << WT_TCL_BIT_POS,	// structure
    RT_TCL_SZ2_B16 = WT_TCL_SZ2_B16 | WT_NEW,  // two-byte, restricted type
    RT_TCL_SZ2_B15 = WT_TCL_SZ2_B15 | WT_NEW,  // two-byte, restricted type
    RT_TCL_SZ2_B12 = WT_TCL_SZ2_B12 | WT_NEW,  // two-byte, restricted type
    RT_TCL_SZ2_B10 = WT_TCL_SZ2_B10 | WT_NEW,  // two-byte, restricted type

    // size 4
    WT_TCL_SZ4_B32 = 9 << WT_TCL_BIT_POS,	// 32 bit value
    WT_TCL_SZ4_STR = 11 << WT_TCL_BIT_POS,	// structure
    WT_TCL_SZ4_F26 = 12 << WT_TCL_BIT_POS,	// double point 26.6
    RT_TCL_SZ4_B32 = WT_TCL_SZ4_B32 | WT_NEW,  // four-byte, restricted type

    // variable size
    WT_TCL_SZV_B32 = 14 << WT_TCL_BIT_POS,	// variable size up to 32
    WT_TCL_SZV_B64 = 15 << WT_TCL_BIT_POS,	// variable size up to 64
    WT_TCL_SZV_UINT32 = 16 << WT_TCL_BIT_POS, // variable size up to max uint32 - 8
};

Event create_event( void )
{
    sysError( poolind >= MAXEVENTS, "No more events" );
    return ( poolind++ );
}

Event mk_event( Event e )
{
    if ( e == NOEVENT ) e = create_event();
    return( e );
}

#define PERM_DEFAULT			(0x02)

#define EVMASK		0x0000ffffUL	/* Event part of evmode */

#define WT_MAX_LABEL_SIZE 63

// we do not allow more than 3 user labels per unique label
#define WT_MAX_LABELS				12

#define NOTINLIST (0xffff)

#define PERM_PRIORITY		0x80	// Bit 7

const uint8_t CTAG_TAG = 0;

static int sNumIdsNotSent = 0;

// Flag is true when the Install methods are being called.
bool gModuleInstallsInProgress = false;

bool pilot = true, isCagePresent = true;
bool gRemotePilot = false;

struct LabelData {
    char* uniqueLabel;
    char* userLabel;
};

struct IdData {
    uint16_t			next;			//next pointer for sync list
    uint16_t			type;
    uint8_t				permissions;
    uint8_t				numLabels : 4;
    uint8_t				units : 3;
    uint8_t				locked : 1;
    uint8_t				syncTag;
    uint8_t				rxSyncTag;
    struct LabelData	labels[ WT_MAX_LABELS ];
    char* defaultValue;
};

static struct IdData* sIdData[ MAXEVENTS ];

#include <cstdarg>

int sprintf_s(
    char* buffer,
    size_t /* sizeOfBuffer */,
    const char* format,
    ... )
{
    va_list	args;
    va_start( args, format );

    int rv = vsprintf( buffer, format, args );
    va_end( args );
    return rv;
}

int
sysStrCat_s(
    char* strDestination,
    size_t numberOfElements,
    const char* strSource )
{
    if ( strlen( strSource ) + strlen( strDestination ) < numberOfElements )
    {
        strcat( strDestination, strSource );
        return 0;
    }
    else
    {
        *strDestination = 0;
        return ERANGE;
    }
}


int
sysStrCpy_s(
    char* strDestination,
    size_t numberOfElements,
    const char* strSource )
{
    if ( strlen( strSource ) < numberOfElements )
    {
        strcpy( strDestination, strSource );
        return 0;
    }
    else
    {
        *strDestination = 0;
        return ERANGE;
    }
}


std::unordered_map<std::string, Event> mUniqueLabelToIdMap;
std::unordered_map<Event, std::string> mEventToDefaultMap;

const char* ServerTelemGetDefaultValue( Event id )
{
    const char* defaultValue = "";
    if ( mEventToDefaultMap.find( id ) != mEventToDefaultMap.end() )
    {
        defaultValue = mEventToDefaultMap[ id ].c_str();
    }

    return defaultValue;
}

enum class Priority
{
    Low,
    High,
    None
};


class PriorityStack
{
public:
    static Priority GetPriority();

    PriorityStack( Priority priority );
    ~PriorityStack();

private:
    static Priority mPriority;

    Priority mPreviousPriority;
};

Priority PriorityStack::mPriority = Priority::Low;


PriorityStack::PriorityStack( Priority priority )
{
    mPreviousPriority = mPriority;
    mPriority = priority;
}


PriorityStack::~PriorityStack()
{
    mPriority = mPreviousPriority;
}


Priority PriorityStack::GetPriority()
{
    return mPriority;
}

static std::unordered_map <Event, const char*> sEventNameMap; // used for debug purposes

#include <functional>

// Action
struct ActionData
{
#ifdef ACTION_DEBUG
    Event					event = 0;
    bool					hiPriority = false;
    bool					isEventChangeAction = false;
#endif
    bool					enabled;	// True if action is enabled and ready to execute.
    std::function<void()>	func;		// Function to execute.
#ifdef ACTION_DEBUG
    uint32_t				maxMs = 0;
    uint32_t				count = 0;
    uint64_t				sumMs = 0;
#endif
};

typedef ActionData* Action;

#define MAXACTIONS	80000

static int32_t sActionsLeft = 0;				// Number of actions left to be created.
static Action sFirstAction;
static Action sNextAction;

static Action CreateAction( std::function<void()> func, bool enable )
{
    sysError( func == nullptr, "Bad func (what's the point?)" );
    sysError( sActionsLeft == 0, "No more actions" );

    // Get a new action from the predefined array.
    Action a = sNextAction++;
    sActionsLeft--;

    // Setup the new action;
    a->func = func;
    a->enabled = enable;

    return a;
}

static void AttachAction( Event e, Action a ) {}

Action create_action( Event e, std::function<void()> func )
{
    Action a = CreateAction( func, false );
    AttachAction( e, a );
    return a;
}

// Enable/Disable the action.
void set_action_enable( Action a, bool enable )
{
    if ( a ) a->enabled = enable;
}

// Create an Action from any function that returns void.
Action CreateAction( Event e, std::function<void()> func, bool enable = true )
{
    Action a = create_action( e, func );
    set_action_enable( a, enable );
    return a;
}


// Create an Action from any old style function (i.e. void function(void* arg)).
Action CreateAction( Event e, void( *func )( void* ), void* arg, bool enable = true )
{
    return CreateAction( e, [ = ]() {( *func )( arg ); }, enable );
}


// Create a new enabled action from any old style function (i.e. void function(void* arg)).
Action create_enable_action( Event e, void( *func ) ( void* ), void* param )
{
    return CreateAction( e, func, param, true );
}

// used only with DoEventChange; helps with ACTION_DEBUG
void special_enable_action( Event e, void ( *func ) ( void* ), void* arg, const char* label )
{
    sEventNameMap.emplace( e, label );
    Action a = create_enable_action( e, func, arg );
#ifdef ACTION_DEBUG
    a->isEventChangeAction = true;
#else
    (void)a; // unused
#endif
}

static void DoEventChange( void* arg ) {}

#define Assert(_Expression) assert(_Expression);

// Insures that the event type is set for registered variables
static void ServerSetEventType( Event id, uint16_t type )
{
    switch ( type & WT_TCL_MASK )
    {
        case WT_TCL_SZ0_B1:		// boolean value
        case WT_TCL_SZ1_B8:		// 1 byte value
        case WT_TCL_SZ1_UN:		// 1 byte value
        case WT_TCL_SZ1_EN:		// 1 byte value
        case WT_TCL_SZ2_B16:	// 2 byte value
        case WT_TCL_SZ2_B15:	// 2 byte value
        case WT_TCL_SZ2_B12:	// 2 byte value
        case WT_TCL_SZ2_B10:	// 2 byte value
        case WT_TCL_SZ2_STR:	// 2 byte value
        case WT_TCL_SZ4_B32:	// 4 byte value
        case WT_TCL_SZ4_STR:	// 4 byte value
            SetEventType( id, legacyType ); break;

        case RT_TCL_SZ0_B1:		// boolean value, restricted
            SetEventType( id, legacyTypeBool ); break;

        case RT_TCL_SZ1_B8:		// 1 byte value, restricted
        case RT_TCL_SZ1_UN:		// 1 byte value
        case RT_TCL_SZ1_EN:		// 1 byte value
            SetEventType( id, legacyTypeShort ); break;

        case RT_TCL_SZ2_B16:	// 2 byte value, restricted
        case RT_TCL_SZ2_B15:	// 2 byte value
        case RT_TCL_SZ2_B12:	// 2 byte value
        case RT_TCL_SZ2_B10:	// 2 byte value
            SetEventType( id, legacyTypeShort ); break;

        case RT_TCL_SZ4_B32:	// 4 byte value, restricted
            SetEventType( id, legacyTypeLong ); break;

        case WT_TCL_SZ4_F26:	// 8 byte value
            SetEventType( id, doubleType ); break;

        case WT_TCL_SZV_B32:	// variable size up to 32 bytes
            SetEventType( id, block32Type ); break;

        case WT_TCL_SZV_B64:	// variable size up to 64 bytes
            SetEventType( id, block64Type ); break;

        case WT_TCL_SZV_UINT32:		// variable size up to max uint32 - 8
            SetEventType( id, blockUInt32Type ); break;

        default:
            Assert( 0 );
            break;
    }
}

void ServerTelemRegister( const char* uniqueLabel, const char* userLabel, uint32_t eventmode, uint16_t type, uint8_t units, uint8_t permissions )
{
    sysError( gModuleInstallsInProgress, "Registering events is not allowed at this time." );
    if ( pilot || gRemotePilot )
    {
        size_t	length;
        struct IdData* node;
        char* mUniqueLabel;
        char* mUserLabel;
        const char* defaultValue;
        Event id = (Event)( eventmode & EVMASK );

        if ( id <= 0 || id >= MAXEVENTS )
            sysFatalError( "Bad ID in ServerTelemRegister" );
        if ( uniqueLabel == nullptr || *uniqueLabel == 0 )
            sysFatalError( "Unique label can't be nullptr" );
        if ( strlen( uniqueLabel ) > WT_MAX_LABEL_SIZE )
            sysFatalError( "Unique label is too long" );
        if ( userLabel != nullptr && strlen( userLabel ) > WT_MAX_LABEL_SIZE )
            sysFatalError( "User label is too long" );
        for ( int i = 0; uniqueLabel[ i ] != '\0'; ++i )
        {
            // ORCA-4320: disallowed characters in MQTT, ORCA-4530: disallow commas
            if ( strchr( "+$*#/,", uniqueLabel[ i ] ) != nullptr )
            {
                char msg[ 128 ];
                snprintf( msg, sizeof( msg ), "Unique label (%s) contains invalid character", uniqueLabel );
                sysFatalError( msg );
            }
        }
        if ( !sIdData[ id ] )
        {
            sIdData[ id ] = new IdData;
            sIdData[ id ]->next = NOTINLIST;
            sIdData[ id ]->syncTag = CTAG_TAG;
            sIdData[ id ]->rxSyncTag = CTAG_TAG;
        }
        node = sIdData[ id ];
        if ( node->numLabels == 0 )		// First registration.
        {
            node->type = ( type & RT_TCL_MASK );
            if ( node->type == WT_TCL_SZ0_B1 ) permissions |= PERM_PRIORITY;		// All boolean values are marked as high priority.
            node->units = units;
            if ( !node->locked ) node->permissions = permissions;
            node->next = NOTINLIST;
            sNumIdsNotSent++;
        }
        else if ( node->numLabels > 0 )		// ID has been registered before.
        {
            {
                auto labelId = mUniqueLabelToIdMap.find( uniqueLabel );
                if ( labelId != mUniqueLabelToIdMap.end() )
                {
                    Event prevId = labelId->second;
                    if ( prevId != id )
                    {
                        char msg[ 256 ];
                        snprintf( msg, sizeof( msg ), "Event %d registering with label (%s), label already used by event %d", id, uniqueLabel, prevId );
                        sysFatalError( msg );
                    }
                }
            }

            if ( node->type != ( type & RT_TCL_MASK ) )
            {
                char msg[ 128 ];
                sprintf_s( msg, sizeof( msg ), "Type mismatch in ServerTelemRegister for unique label: %s", uniqueLabel );
                sysFatalError( msg );
            }

            if ( node->units != units )
            {
                char msg[ 128 ];
                sprintf_s( msg, sizeof( msg ), "Units mismatch in ServerTelemRegister for unique label : %s", uniqueLabel );
                sysFatalError( msg );
            }

            if ( !node->locked ) node->permissions = permissions;
        }
        else
        {
            sysFatalError( "Bad numLabels" );
        }

        defaultValue = ServerTelemGetDefaultValue( id );

        int32_t defaultSize = strlen( defaultValue ) + 1;
        if ( defaultSize > 1 )
        {
            node->defaultValue = new char[ defaultSize ];
            sysStrCpy_s( node->defaultValue, defaultSize, defaultValue );
        }

        length = strlen( uniqueLabel ) + 1;
        mUniqueLabel = new char[ length ];
        sysStrCpy_s( mUniqueLabel, length, uniqueLabel );
        if ( node->numLabels >= WT_MAX_LABELS )
        {
            char msg[ 256 ] = "Need to increase WT_MAX_LABELS for unique label: ";
            sysStrCat_s( msg, sizeof( msg ), uniqueLabel );
            sysFatalError( msg );
        }
        node->labels[ node->numLabels ].uniqueLabel = mUniqueLabel;

        sysError( mUniqueLabelToIdMap[ mUniqueLabel ] != 0 && mUniqueLabelToIdMap[ mUniqueLabel ] != id, "Name is already attached to another event" );
        mUniqueLabelToIdMap[ mUniqueLabel ] = id;

        if ( userLabel == nullptr )
        {
            mUserLabel = nullptr;
            node->labels[ node->numLabels ].userLabel = nullptr;
        }
        else
        {
            length = strlen( userLabel ) + 1;
            mUserLabel = new char[length];
            sysStrCpy_s( mUserLabel, length, userLabel );
            node->labels[ node->numLabels ].userLabel = mUserLabel;
        }

        ++node->numLabels;

        if ( node->numLabels == 1 )
        {
            PriorityStack ps( Priority::Low );
            special_enable_action( id, DoEventChange, reinterpret_cast<void*>( id ), mUniqueLabel );
        }

        // Set the event type as soon as the variable is registered.
        ServerSetEventType( id, type );
    }
} // end ServerTelemRegister

Event Register( std::string name, Event event, uint16_t type, uint8_t units, uint8_t permissions )
{
    sysError( event == NOEVENT && ( type & WT_TCL_MASK ) == WT_TCL_SZV_UINT32, "This event type can't be created automatically" );

    // Create a new event if one is not provided.
    if ( event == NOEVENT ) event = mk_event( event );

    // Register with server.
    ServerTelemRegister( name.c_str(), nullptr, event, type, units, permissions );

    return event;
}

enum EventIds
{
    __EPOOLSTART = 18255
};

void init_scheduler( void )
{
#if 0
    // Allocate the queues.
    sHighQueue.queueNodes = (QueueNode*)getmem( MAXEVENTS * sizeof( QueueNode ) );
    sLowQueue.queueNodes = (QueueNode*)getmem( MAXEVENTS * sizeof( QueueNode ) );
    // Allocate the action nodes for action lists.
    sFirstActionNode = sNextActionNode = (ActionNode*)getmem( MAXACTIONS * sizeof( ActionNode ) );
    sActionNodesLeft = MAXACTIONS;
#endif

    // Allocate the actions.
    sFirstAction = sNextAction = new ActionData[ MAXACTIONS ];
    sActionsLeft = MAXACTIONS;
}

#include <fstream>
#include <sstream>
#include <system_error>
#include <ctime>
#include <chrono>
#include <iomanip>

using std::ofstream;
using std::ostringstream;
using std::system_error;
using std::system_category;

namespace
{
    std::string datestamp()
    {
        using namespace std::chrono;

        // get current time
        auto now = system_clock::now();

        // get number of milliseconds for the current second
        // (remainder after division into seconds)
        auto ms = duration_cast<milliseconds>( now.time_since_epoch() ) % 1000;

        // convert to std::time_t in order to convert to std::tm (broken time)
        auto timer = system_clock::to_time_t( now );

        // convert to broken time
        std::tm bt = *std::localtime( &timer );

        std::ostringstream oss;

        oss << std::put_time( &bt, "%Y.%m.%d-%H:%M:%S" );
        oss << '.' << std::setfill( '0' ) << std::setw( 3 ) << ms.count();

        return oss.str();
    }

    constexpr auto logfilename = "event.system.thread.safety.test.log";
    void log( const std::string& s )
    {
        ofstream f{ logfilename, std::ios::app };
        if ( !f )
        {
            ostringstream ss;
            ss << "Failed to open file \"" << logfilename << "\"";
            throw system_error{ errno, system_category(), ss.str() };
        }
        f << "[" << datestamp() << "] " << s << "\n";
    }
} // namespace

#include <iostream>
#include <thread>
#include <filesystem>

int main()
{
    // Delete log file at start so it doesn't accumulate with each run and eat up disk space.
    std::filesystem::remove( logfilename );

    init_events( __EPOOLSTART );
    init_scheduler();

    Event eSubscribeCount = Register( "Foo.SubscribeCount", NOEVENT, WT_SINT32, WT_UNITS_NONE, PERM_DEFAULT );
    Event ePublishCount   = Register( "Foo.PublishCount"  , NOEVENT, WT_SINT32, WT_UNITS_NONE, PERM_DEFAULT );

    auto f1 = [ eSubscribeCount ] {
        const auto x = getlevent( eSubscribeCount );

        // Write result to file so the compiler can't optimize out the above getlevent call:
        log( std::to_string( x ) );
    };
    
    auto f2 = [ ePublishCount, eSubscribeCount ] {
        inclevent( ePublishCount );

        inclevent( eSubscribeCount ); // This is one of the causes of the data race.
                                      // This thread writes the event's value while
                                      // the other thread reads this event's value.
    };

    std::thread t1{ f1 };
    std::thread t2{ f2 };
    if ( t1.joinable() ) { t1.join(); }
    if ( t2.joinable() ) { t2.join(); }

    std::cerr << "End of program.\n";
}
