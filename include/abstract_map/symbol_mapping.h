#include <list>
#include <string>

#include <tinyxml.h>

namespace humancues {
  enum MappingType { LABEL, INFO, EMPTY };

  class SymbolMapping {
    public:
      unsigned int tag_id_;
      MappingType type_;
      std::string text_;

      SymbolMapping(const unsigned int &tag_id, const std::string &type, const std::string &text);

      static bool parseListFromXML(const char* filename, std::list<SymbolMapping> * const out);
  };
} // humancues

