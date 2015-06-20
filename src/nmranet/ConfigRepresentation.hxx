/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file ConfigRepresentation.hxx
 *
 * Static representation of a config file.
 *
 * @author Balazs Racz
 * @date 31 May 2014
 */

#ifndef _NMRANET_CONFIGREPRESENTATION_HXX_
#define _NMRANET_CONFIGREPRESENTATION_HXX_

#include "nmranet/ConfigEntry.hxx"
#include "nmranet/MemoryConfig.hxx"

namespace nmranet
{

/// Starts a CDI group.
///
/// @param group is the c++ name of the struct that is being defined.
/// @param base is a c++ name which represents the beginning of the chain.
/// @param ARGS are additional arguments for group options, like Name(...),
/// Description(...), Segment(...), Offset(...) or MainCdi().
#define BEGIN_GROUP(group, base, ARGS...)                                      \
    class group##base : public nmranet::BaseGroup                              \
    {                                                                          \
    public:                                                                    \
        using base_type = BaseGroup;                                           \
        using base_type::base_type;                                            \
        using Name = AtomConfigOptions::Name;                                  \
        using Description = AtomConfigOptions::Description;                    \
        using Segment = GroupConfigOptions::Segment;                           \
        using Offset = GroupConfigOptions::Offset;                             \
        using MainCdi = GroupConfigOptions::MainCdi;                           \
        static constexpr GroupConfigOptions group_opts()                       \
        {                                                                      \
            return GroupConfigOptions(ARGS);                                   \
        }                                                                      \
        void render_cdi(std::string *s) const                                  \
        {                                                                      \
        }                                                                      \
    };

/// Helper class for starting a group. Terminates the type recursion.
class BaseGroup : public nmranet::ConfigReference
{
public:
    using ConfigReference::ConfigReference;
    static constexpr unsigned size()
    {
        return 0;
    }
};

/// Adds an entry to a CDI group.
///
/// @param group is the c++ name of the struct that is being defined.
/// @param prev_entry_name links to the name of the previous entry
/// @param entry_name defines the name of the current entry
/// @param type defines the c++ class / struct of the entry being added
/// @param ARGS are additional arguments for the entry options, like Name(...),
/// Description(...). If a subgroup is added, then group options are also
/// allowed and they will override the respective values from the group
/// definition.
#define EXTEND_GROUP(group, prev_entry_name, entry_name, type, ARGS...)        \
    class group##entry_name : public group##prev_entry_name                    \
    {                                                                          \
    public:                                                                    \
        using base_type = group##prev_entry_name;                              \
        using current_type = type;                                             \
        using base_type::base_type;                                            \
        using Name = AtomConfigOptions::Name;                                  \
        using Description = AtomConfigOptions::Description;                    \
        static constexpr unsigned size()                                       \
        {                                                                      \
            return current_type::size() + offset_from_base();                  \
        }                                                                      \
        static constexpr unsigned offset_from_base()                           \
        {                                                                      \
            return base_type::size();                                          \
        }                                                                      \
        constexpr unsigned last_offset()                                       \
        {                                                                      \
            return offset() + offset_from_base();                              \
        }                                                                      \
        constexpr current_type entry_name()                                    \
        {                                                                      \
            static_assert(!group_opts().is_cdi() ||                            \
                    current_type(0).group_opts().is_segment(),                 \
                "May only have segments inside CDI.");                         \
            return group_opts().is_cdi()                                       \
                ? current_type(                                                \
                      current_type(0).group_opts().get_segment_offset())       \
                : current_type(last_offset());                                 \
        }                                                                      \
        void render_cdi(std::string *s) const                                  \
        {                                                                      \
            base_type::render_cdi(s);                                          \
            entry_name().config_renderer().render_cdi(s, ##ARGS);              \
        }                                                                      \
    };

/// Finalizes a CDI group definition.
///
/// @param group is the c++ name of the struct that is being defined.
/// @param last_entry_name links to the name of the last entry
///
#define END_GROUP(group, last_entry_name)                                      \
    class group : public group##last_entry_name                                \
    {                                                                          \
    public:                                                                    \
        using base_type = group##last_entry_name;                              \
        using base_type::base_type;                                            \
        void render_content_cdi(std::string *s) const                          \
        {                                                                      \
            base_type::render_cdi(s);                                          \
        }                                                                      \
        constexpr GroupConfigRenderer<group> config_renderer()                 \
        {                                                                      \
            return GroupConfigRenderer<group>(1, *this);                       \
        }                                                                      \
    };

/// Defines a repeated group of a given type and a given number of repeats.
///
/// Typical usage:
///
///  using AllConsumers = RepeatedGroup<ConsumerConfig, 3>;
///
/// then add AllConsumers as an entry to the enclosing group or segment.
template <class Group, unsigned N> class RepeatedGroup : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    using base_type::base_type;
    static constexpr unsigned size()
    {
        return Group::size() * N;
    }
    template <int K> constexpr Group entry()
    {
        static_assert(K < N, "Tried to fetch an entry of a repeated "
                             "group that does not exist!");
        return Group(offset_ + (K * Group::size()));
    }
    constexpr GroupConfigRenderer<Group> config_renderer()
    {
        return GroupConfigRenderer<Group>(N, entry<0>());
    }
};

///
/// Defines an empty group with no members, but blocking a certain amount of
/// space in the rendered configuration.
///
template <unsigned N> class EmptyGroup : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    using base_type::base_type;
    static constexpr unsigned size()
    {
        return N;
    }
    constexpr EmptyGroupConfigRenderer config_renderer()
    {
        return EmptyGroupConfigRenderer(N);
    }
};

/// Base class for all entries that can appear in the MainCdi group. THe common
/// property of these entries is that they do not rely on the offset/size
/// propagation of the previous entries, because they either do not take part
/// in the layout algorithm (e.g. the <identification> tag) or they specify the
/// origin explcitly.
class ToplevelEntryBase : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    using base_type::base_type;
    static constexpr GroupConfigOptions group_opts()
    {
        return GroupConfigOptions(GroupConfigOptions::Segment(1000));
    }
    static constexpr unsigned size()
    {
        return 0;
    }
};

/// Add this entry to the beginning of the CDI group to render an
/// <identification> tag at the beginning of the output cdi.xml. Requires a
/// global symbol of @ref nmranet::SNIP_STATIC_DATA to fill in the specific
/// values of the identification tree.
class Identification : public ToplevelEntryBase
{
public:
    using base_type = ToplevelEntryBase;
    using base_type::base_type;
    static constexpr IdentificationRenderer config_renderer()
    {
        return IdentificationRenderer();
    }
};

/// Renders an <acdi> tag in the CDI group.
class Acdi : public ToplevelEntryBase
{
public:
    using base_type = ToplevelEntryBase;
    using base_type::base_type;
    static constexpr AcdiRenderer config_renderer()
    {
        return AcdiRenderer();
    }
};

BEGIN_GROUP(UserInfoSegment, base, Segment(MemoryConfigDefs::SPACE_ACDI_USR),
    Offset(1));
EXTEND_GROUP(
    UserInfoSegment, base, name, StringConfigEntry<63>, //
    Name("User name"),                                  //
    Description(
        "This name will appear in network browsers for the current node."));
EXTEND_GROUP(UserInfoSegment, name, description, StringConfigEntry<64>, //
    Name("User description"),                                           //
    Description("This description will appear in network browsers for the "
                "current node."));
/// Configuration description for a segment containing the ACDI user-modifiable
/// data. The implementation refers to the ACDI-userdata space number and does
/// not depend on where the actual data is located.
END_GROUP(UserInfoSegment, description);

} // namespace nmranet

#endif // _NMRANET_CONFIGREPRESENTATION_HXX_