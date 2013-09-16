#ifndef _OPENCV2FX_H
#define _OPENCV2FX_H

#include "cv.h"
#include "ofxImageEffect.h"
#include "ofxMemory.h"
#include "ofxMultiThread.h"
#include "ofxPixels.h"

#define PLUGIN_GROUPING "www.bratwurstandhaggis.org.uk"

// defines a new control for the plugin which controls a floating point variable
void defineDoubleParam( OfxPropertySuiteV1 *gPropHost,
			OfxParameterSuiteV1 *gParamHost,
			OfxParamSetHandle effectParams,
			const char *name,
			const char *label,
			const char *hint,
			double min,
			double max,
			double initial);


template <class T> inline T 
clamp(T v, int min, int max)
{
  if(v < T(min)) return T(min);
  if(v > T(max)) return T(max);
  return v;
}


/** @brief The core 'OFX Support' namespace, used by plugin implementations. All code for these are defined in the common support libraries.
 */
namespace OFX {
    /** @brief maps a status to a string for debugging purposes, note a c-str for printf */
    const char * mapStatusToString(OfxStatus stat);

    /** @brief namespace for OFX support lib exceptions, all derive from std::exception, calling it */
    namespace Exception {

        /** @brief thrown when a suite returns a dud status code
         */
        class Suite : public std::exception {
            protected :
            OfxStatus _status;
            public :
            Suite(OfxStatus s) : _status(s) {}
            OfxStatus status(void) {return _status;}
            operator OfxStatus() {return _status;}

            /** @brief reimplemented from std::exception */
            virtual const char * what () const throw () {return mapStatusToString(_status);}

        };

        /** @brief Exception indicating that a host doesn't know about a property that is should do */
        class PropertyUnknownToHost : public std::exception {
            protected :
            std::string _what;
            public :
            PropertyUnknownToHost(const char *what) : _what(what) {}
            virtual ~PropertyUnknownToHost() throw() {}

            /** @brief reimplemented from std::exception */
            virtual const char * what () const throw ()
            {
                return _what.c_str();
            }
        };

        /** @brief exception indicating that the host thinks a property has an illegal value */
        class PropertyValueIllegalToHost : public std::exception {
            protected :
            std::string _what;
            public :
            PropertyValueIllegalToHost(const char *what) : _what(what) {}
            virtual ~PropertyValueIllegalToHost() throw() {}

            /** @brief reimplemented from std::exception */
            virtual const char * what () const throw ()
            {
                return _what.c_str();
            }
        };

        /** @brief exception indicating a request for a named thing exists (eg: a param), but is of the wrong type, should never make it back to the main entry
         indicates a logical error in the code. Asserts are raised in debug code in these situations.
         */
        class TypeRequest : public std::exception {
            protected :
            std::string _what;
            public :
            TypeRequest(const char *what) : _what(what) {}
            virtual ~TypeRequest() throw() {}

            /** @brief reimplemented from std::exception */
            virtual const char * what () const throw ()
            {
                return _what.c_str();
            }
        };

        ////////////////////////////////////////////////////////////////////////////////
        // These exceptions are to be thrown by the plugin if it hits a problem, the
        // code managing the main entry will trap the exception and return a suitable
        // status code to the host.

        /** @brief exception indicating a required host feature is missing */
        class HostInadequate : public std::exception {
            protected :
            std::string _what;
            public :
            HostInadequate(const char *what) : _what(what) {}
            virtual ~HostInadequate() throw() {}
            
            /** @brief reimplemented from std::exception */
            virtual const char * what () const throw ()
            {
                return _what.c_str();
            }
        };
        
    }; // end of Exception namespace

    /** @brief Throws an @ref OFX::Exception::Suite depending on the status flag passed in */
    void
    throwSuiteStatusException(OfxStatus stat)
    throw(OFX::Exception::Suite, std::bad_alloc);

    void
    throwHostMissingSuiteException(std::string name)
    throw(OFX::Exception::Suite);
};

#endif
