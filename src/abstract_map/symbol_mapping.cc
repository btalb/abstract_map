#include "abstract_map/symbol_mapping.h"

namespace humancues {
MappingType getType(const std::string &type) {
  if (type == "LABEL") {
    return LABEL;
  } else if (type == "INFO") {
    return INFO;
  }
  return EMPTY;  // default
}

SymbolMapping::SymbolMapping(const unsigned int &tag_id,
                             const std::string &type, const std::string &text) {
  tag_id_ = tag_id;
  type_ = getType(type);
  text_ = text;
}

bool SymbolMapping::parseListFromXML(const char *filename,
                                     std::list<SymbolMapping> *const out) {
  // Attempt to load the XML file
  TiXmlDocument doc(filename);
  if (!doc.LoadFile()) return false;

  // Get the document
  TiXmlHandle hDoc(&doc);
  TiXmlElement *eMaps, *eMap;

  // Find the start of the "mappings" element
  eMaps = hDoc.FirstChildElement("mappings").Element();
  if (!eMaps) return false;

  // Loop through, creating each of the symbol mappings
  eMap = eMaps->FirstChildElement("mapping");
  for (; eMap; eMap = eMap->NextSiblingElement()) {
    // Get the attributes
    int tag_id;
    std::string type, text;
    eMap->QueryIntAttribute("tag_id", &tag_id);
    eMap->QueryStringAttribute("type", &type);
    eMap->QueryStringAttribute("text", &text);

    // Create the mapping object, add it to the list
    humancues::SymbolMapping sm((unsigned int)tag_id, type, text);
    out->push_back(sm);
  }

  return true;
}
}  // namespace humancues
