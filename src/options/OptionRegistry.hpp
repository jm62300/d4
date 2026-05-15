#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "src/options/OptionBase.hpp"

namespace d4
{

  /**
   * @brief Registry to manage dot-notation paths for options.
   */
  class OptionRegistry
  {
  public:
    /** @brief Register an option with a specific path. */
    void registerOption(const std::string &path, OptionBase *option)
    {
      m_options[path] = option;
    }

    /** @brief Check if an option exists. */
    bool hasOption(const std::string &path) const
    {
      return m_options.count(path) > 0;
    }

    /** @brief Set an option from a string value by propagating it from its parent group. */
    void setOption(const std::string &path, const std::string &value)
    {
      std::string groupPath;
      std::string optName;
      auto lastDot = path.find_last_of('.');

      if (lastDot == std::string::npos)
      {
        groupPath = ""; // Root
        optName = path;
      }
      else
      {
        groupPath = path.substr(0, lastDot);
        optName = path.substr(lastDot + 1);
      }

      auto itGroup = m_groups.find(groupPath);
      if (itGroup != m_groups.end())
      {
        itGroup->second->setPropagation(optName, value);
      }
      else if (groupPath == "")
      {
        // Broadcast to all top-level groups
        for (const auto &[gPath, group] : m_groups)
        {
          if (gPath.find('.') == std::string::npos)
          {
            group->setPropagation(optName, value);
          }
        }
      }
    }

    /** @brief Set multiple options from a vector of pairs, sorted by depth. */
    void setOptions(std::vector<std::pair<std::string, std::string>> options)
    {
      // Sort options by depth (number of dots) to ensure top-to-bottom order
      std::stable_sort(options.begin(), options.end(),
                       [](const auto &a, const auto &b)
                       {
                         auto countDots = [](const std::string &s)
                         {
                           return (size_t)std::count(s.begin(), s.end(), '.');
                         };
                         return countDots(a.first) < countDots(b.first);
                       });

      for (const auto &opt : options)
      {
        setOption(opt.first, opt.second);
      }
    }

    /** @brief Parse command line arguments and apply them. */
    void parseArgv(int argc, char *argv[])
    {
      std::vector<std::pair<std::string, std::string>> collectedOptions;
      for (int i = 1; i < argc; ++i)
      {
        std::string arg = argv[i];
        if (arg.size() > 2 && arg.substr(0, 2) == "--")
        {
          std::string key, value;
          auto eqPos = arg.find('=');
          if (eqPos != std::string::npos)
          {
            key = arg.substr(2, eqPos - 2);
            value = arg.substr(eqPos + 1);
          }
          else if (i + 1 < argc)
          {
            key = arg.substr(2);
            std::string next = argv[i + 1];
            if (next.size() < 2 || next.substr(0, 2) != "--")
            {
              value = next;
              ++i;
            }
            else
            {
              value = "true";
            }
          }
          else
          {
            key = arg.substr(2);
            value = "true";
          }
          collectedOptions.push_back({key, value});
        }
      }

      setOptions(collectedOptions);
    }

    /** @brief Display a pretty tree of all registered options. */
    void displayHelp(std::ostream &out) const
    {
      out << "\n\033[1;36mAvailable Options (use "
             "--path.to.option=value):\033[0m\n";
      std::set<std::string> displayed_nodes;
      renderTree(out, "", "", displayed_nodes);
      out << std::endl;
    }

    void renderTree(std::ostream &out, const std::string &prefix,
                    const std::string &currentPath,
                    std::set<std::string> &displayed) const
    {
      std::map<std::string, bool> children;
      for (const auto &[path, opt] : m_options)
      {
        if (currentPath.empty() || (path.size() > currentPath.size() &&
                                    path.substr(0, currentPath.size() + 1) ==
                                        currentPath + "."))
        {
          std::string remaining =
              currentPath.empty() ? path : path.substr(currentPath.size() + 1);
          auto dotPos = remaining.find('.');
          std::string name = (dotPos == std::string::npos)
                                 ? remaining
                                 : remaining.substr(0, dotPos);
          children[name] = (dotPos == std::string::npos);
        }
      }

      for (const auto &[name, isLeaf] : children)
      {
        std::string fullPath =
            currentPath.empty() ? name : currentPath + "." + name;
        out << prefix << "└── " << name;
        if (isLeaf)
        {
          OptionBase *opt = m_options.at(fullPath);
          out << " \033[90m(" << opt->getTypeName() << ")\033[0m : "
              << opt->getDescription() << " \033[33m[default: "
              << opt->getValueAsString() << "]\033[0m";
          std::string possible = opt->getPossibleValues();
          if (!possible.empty())
            out << " \033[94m{" << possible << "}\033[0m";
          out << "\n";
        }
        else
        {
          out << "/\n";
          renderTree(out, prefix + "    ", fullPath, displayed);
        }
      }
    }

    const std::map<std::string, OptionBase *> &getOptions() const
    {
      return m_options;
    }

    void registerGroup(const std::string &path, OptionBase *group)
    {
      m_groups[path] = group;
    }

  private:
    std::map<std::string, OptionBase *> m_options;
    std::map<std::string, OptionBase *> m_groups;
  };

} // namespace d4
