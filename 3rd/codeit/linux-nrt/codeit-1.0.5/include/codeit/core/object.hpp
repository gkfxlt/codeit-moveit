﻿#ifndef CODEIT_CORE_OBJECT_H_
#define CODEIT_CORE_OBJECT_H_

#include <vector>
#include <memory>
#include <functional>

#include <codeit/core/basictype.hpp>
#include <codeit/core/tinyxml2.h>
#include <codeit/core/log.hpp>

namespace codeit::core
{
	using XmlDocument = tinyxml2::XMLDocument;
	using XmlDeclaration = tinyxml2::XMLDeclaration;
	using XmlNode = tinyxml2::XMLNode;
	using XmlElement = tinyxml2::XMLElement;
	using XmlAttribute = tinyxml2::XMLAttribute;

	template<typename T> class ImpPtr
	{
	public:
		auto reset(T* p)->void { data_unique_ptr_.reset(p); }
		auto get()const->const T* { return data_unique_ptr_.get(); }
		auto get()->T* { return data_unique_ptr_.get(); }
		auto operator->()const->const T* { return data_unique_ptr_.get(); }
		auto operator->()->T* { return data_unique_ptr_.get(); }
		auto operator*()const->const T& { return *data_unique_ptr_; }
		auto operator*()->T& { return *data_unique_ptr_; }

		~ImpPtr() = default;
		explicit ImpPtr(T *data_ptr) :data_unique_ptr_(data_ptr) {}
		explicit ImpPtr() :data_unique_ptr_(new T) {}
		ImpPtr(const ImpPtr &other) :data_unique_ptr_(new T(*other.data_unique_ptr_)) {}
		ImpPtr(ImpPtr &&other)noexcept = default;
		ImpPtr& operator=(const ImpPtr &other) { *data_unique_ptr_ = *other.data_unique_ptr_; return *this; }
		ImpPtr& operator=(ImpPtr &&other)noexcept = default;

	private:
		std::unique_ptr<T> data_unique_ptr_;
	};
	template <class T, class A = std::allocator<T> >class ImpContainer
	{
	public:
		using allocator_type = A;
		using value_type = typename std::allocator_traits<A>::value_type;
		using reference = T&;
		using const_reference = const T&;
		using pointer = typename std::allocator_traits<A>::pointer;
		using const_pointer = typename std::allocator_traits<A>::const_pointer;
		using difference_type = typename std::allocator_traits<A>::difference_type;
		using size_type = typename std::allocator_traits<A>::size_type;
		class iterator;
		class const_iterator;

		class iterator
		{
		public:
			using difference_type = typename ImpContainer::difference_type;
			using value_type = typename ImpContainer::value_type;
			using reference = typename ImpContainer::reference;
			using pointer = typename ImpContainer::pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const iterator&other)->iterator& = default;
			auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
			auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->iterator& { ++iter_; return *this; }
			auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
			auto operator--()->iterator& { --iter_; return *this; } //optional
			auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->iterator { return iter_ + size; } //optional
			friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
			auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->iterator { return iter_ - size; } //optional
			auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->reference { return iter_->operator*(); }
			auto operator->() const->pointer { return iter_->operator->(); }
			auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

			~iterator() = default;
			iterator() = default;
			iterator(const iterator& other) = default;
			iterator(const typename std::vector<ImpPtr<T>>::iterator iter) :iter_(iter) {} //

		private:
			friend class ImpContainer<T, A>::const_iterator;
			friend class ImpContainer<T, A>;
			typename std::vector<ImpPtr<T>>::iterator iter_;
		};
		class const_iterator
		{
		public:
			using difference_type = typename ImpContainer::difference_type;
			using value_type = typename ImpContainer::value_type;
			using const_reference = typename ImpContainer::const_reference;
			using const_pointer = typename ImpContainer::const_pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const const_iterator&)->const_iterator& = default;
			auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
			auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->const_iterator& { ++iter_; return *this; }
			auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
			auto operator--()->const_iterator& { --iter_; return *this; } //optional
			auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->const_iterator { return iter_ + size; } //optional
			friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return size + iter.iter_; } //optional
			auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->const_iterator { return iter_ - size; } //optional
			auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->const_reference { return iter_->operator*(); }
			auto operator->() const->const_pointer { return iter_->operator->(); }
			auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

			~const_iterator() = default;
			const_iterator() = default;
			const_iterator(const const_iterator&) = default;
			const_iterator(const iterator& other) :iter_(other.iter_) {}
			const_iterator(const typename std::vector<ImpPtr<T>>::const_iterator iter) :iter_(iter) {} //

		private:
			typename std::vector<ImpPtr<T>>::const_iterator iter_;
		};
		using reverse_iterator = std::reverse_iterator<iterator>; //optional
		using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

		auto size()const->size_type { return container_.size(); }
		auto max_size()->size_type { return container_.max_size(); }
		auto empty()->bool { return container_.empty(); }

		auto begin()->iterator { return container_.begin(); }
		auto begin() const->const_iterator { return container_.begin(); }
		auto cbegin() const->const_iterator { return container_.cbegin(); }
		auto end()->iterator { return container_.end(); }
		auto end() const->const_iterator { return container_.end(); }
		auto cend() const->const_iterator { return container_.cend(); }
		auto rbegin()->reverse_iterator { return container_.rbegin(); } //optional
		auto rbegin() const->const_reverse_iterator { return container_.rbegin(); } //optional
		auto crbegin() const->const_reverse_iterator { return container_.crbegin(); } //optional
		auto rend()->reverse_iterator { return container_.rend(); } //optional
		auto rend() const->const_reverse_iterator { return container_.rend(); } //optional
		auto crend() const->const_reverse_iterator { return container_.crend(); } //optional

		auto front()->reference { return *begin(); } //optional
		auto front() const->const_reference { return *begin(); } //optional
		auto back()->reference { return *(end() - 1); } //optional
		auto back() const->const_reference { return *(end() - 1); } //optional
		auto at(size_type size)->reference { return *container_.at(size); } //optional
		auto at(size_type size) const->const_reference { return *container_.at(size); } //optional
		auto operator[](size_type size)->reference { return *container_.operator[](size); } //optional
		auto operator[](size_type size) const->const_reference { return *container_.operator[](size); } //optional

		auto pop_back()->void { container_.pop_back(); } //optional
		auto erase(iterator iter)->iterator { return container_.erase(iter.iter_); } //optional
		auto erase(iterator begin_iter, iterator end_iter)->iterator { return container_.erase(begin_iter.iter_, end_iter.iter_); } //optional
		auto clear()->void { container_.clear(); } //optional

		auto push_back_ptr(T*ptr)->void { container_.push_back(ImpPtr<T>(ptr)); }
		auto swap(ImpContainer& other)->void { return container_.swap(other.container_); }

		~ImpContainer() = default;
		ImpContainer() = default;
		ImpContainer(const ImpContainer&) = default;
		ImpContainer(ImpContainer&&other) = default;
		ImpContainer& operator=(const ImpContainer& other) = default;
		ImpContainer& operator=(ImpContainer&& other) = default;

	private:
		typename std::vector<ImpPtr<T>> container_;
		friend class Object;
	};

	class Object
	{
	public:
		struct TypeInfo
		{
			using DefaultConstructor = std::function<Object*(void)>;
			using CopyConstructor = std::function<Object*(const Object &)>;
			using MoveConstructor = std::function<Object*(Object &&)>;
			using CopyAssign = std::function<Object&(const Object &, Object &)>;
			using MoveAssign = std::function<Object&(Object &&, Object &)>;

			DefaultConstructor default_construct_func;
			CopyConstructor copy_construct_func;
			MoveConstructor move_construct_func;
			CopyAssign copy_assign_func;
			MoveAssign move_assign_func;

			auto registerTo(const std::string &type, Object &object)->void;
			auto registerTo(const std::string &type)->void;
			template<typename ChildType> static auto inline CreateTypeInfo()->TypeInfo
			{
				static_assert(std::is_base_of<Object, ChildType>::value, "failed to register type, because it is not inheritated from Object");

				return TypeInfo
				{
					default_construct_func_<ChildType>(),
					copy_construct_func_<ChildType>(),
					move_construct_func_<ChildType>(),
					copy_assign_func_<ChildType>(),
					move_assign_func_<ChildType>()
				};
			}

		private:
			template<typename ChildType>
			static auto default_constructor_()->Object*
			{
				return new ChildType;
			}
			template<typename ChildType>
			static auto default_construct_func_(std::enable_if_t<std::is_default_constructible<ChildType>::value> *a = nullptr)->DefaultConstructor
			{
				return DefaultConstructor(TypeInfo::default_constructor_<ChildType>);
			}
			template<typename ChildType>
			static auto default_construct_func_(std::enable_if_t<!std::is_default_constructible<ChildType>::value> *a = nullptr)->DefaultConstructor
			{
				return nullptr;
			}

			template<typename ChildType>
			static auto copy_constructor_(const Object &other)->Object*
			{
				if (!dynamic_cast<const ChildType *>(&other))THROW_FILE_LINE("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
				return new ChildType(dynamic_cast<const ChildType &>(other));
			}
			template<typename ChildType>
			static auto copy_construct_func_(std::enable_if_t<std::is_copy_constructible<ChildType>::value> *a = nullptr)->CopyConstructor
			{
				return CopyConstructor(TypeInfo::copy_constructor_<ChildType>);
			}
			template<typename ChildType>
			static auto copy_construct_func_(std::enable_if_t<!std::is_copy_constructible<ChildType>::value> *a = nullptr)->CopyConstructor
			{
				return nullptr;
			}

			template<typename ChildType>
			static auto move_constructor_(Object &&other)->Object*
			{
				if (!dynamic_cast<ChildType *>(&other))THROW_FILE_LINE("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
				return new ChildType(dynamic_cast<ChildType &&>(other));
			}
			template<typename ChildType>
			static auto move_construct_func_(std::enable_if_t<std::is_move_constructible<ChildType>::value> *a = nullptr)->MoveConstructor
			{
				return MoveConstructor(move_constructor_<ChildType>);
			}
			template<typename ChildType>
			static auto move_construct_func_(std::enable_if_t<!std::is_move_constructible<ChildType>::value> *a = nullptr)->MoveConstructor
			{
				return nullptr;
			}

			template<typename ChildType>
			static auto copy_assign_(const Object &from_object, Object &to_object)->Object&
			{
				if (!dynamic_cast<const ChildType *>(&from_object))THROW_FILE_LINE("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
				if (!dynamic_cast<ChildType *>(&to_object))THROW_FILE_LINE("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
				return dynamic_cast<ChildType &>(to_object) = dynamic_cast<const ChildType &>(from_object);
			}
			template<typename ChildType>
			static auto copy_assign_func_(std::enable_if_t<std::is_copy_assignable<ChildType>::value> *a = nullptr)->CopyAssign
			{
				return CopyAssign(copy_assign_<ChildType>);
			}
			template<typename ChildType>
			static auto copy_assign_func_(std::enable_if_t<!std::is_copy_assignable<ChildType>::value> *a = nullptr)->CopyAssign
			{
				return nullptr;
			}

			template<typename ChildType>
			static auto move_assign_(Object &&from_object, Object &to_object)->Object&
			{
				if (!dynamic_cast<ChildType *>(&from_object))THROW_FILE_LINE("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
				if (!dynamic_cast<ChildType *>(&to_object))THROW_FILE_LINE("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
				return dynamic_cast<ChildType &>(to_object) = dynamic_cast<ChildType &&>(from_object);
			}
			template<typename ChildType>
			static auto move_assign_func_(std::enable_if_t<std::is_move_assignable<ChildType>::value> *a = nullptr)->MoveAssign
			{
				return std::function<Object&(Object &&, Object &)>(move_assign_<ChildType>);
			}
			template<typename ChildType>
			static auto move_assign_func_(std::enable_if_t<!std::is_move_assignable<ChildType>::value> *a = nullptr)->MoveAssign
			{
				return nullptr;
			}
		};

	public:
		static auto attributeBool(const core::XmlElement &xml_ele, const std::string &attribute_name)->bool;
		static auto attributeBool(const core::XmlElement &xml_ele, const std::string &attribute_name, bool default_value)->bool;
		static auto attributeInt64(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::int64_t;
		static auto attributeInt64(const core::XmlElement &xml_ele, const std::string &attribute_name, std::int64_t default_value)->std::int64_t;
		static auto attributeInt32(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::int32_t;
		static auto attributeInt32(const core::XmlElement &xml_ele, const std::string &attribute_name, std::int32_t default_value)->std::int32_t;
		static auto attributeInt16(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::int16_t;
		static auto attributeInt16(const core::XmlElement &xml_ele, const std::string &attribute_name, std::int16_t default_value)->std::int16_t;
		static auto attributeInt8(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::int8_t;
		static auto attributeInt8(const core::XmlElement &xml_ele, const std::string &attribute_name, std::int8_t default_value)->std::int8_t;
		static auto attributeUint64(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint64_t;
		static auto attributeUint64(const core::XmlElement &xml_ele, const std::string &attribute_name, std::uint64_t default_value)->std::uint64_t;
		static auto attributeUint32(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint32_t;
		static auto attributeUint32(const core::XmlElement &xml_ele, const std::string &attribute_name, std::uint32_t default_value)->std::uint32_t;
		static auto attributeUint16(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint16_t;
		static auto attributeUint16(const core::XmlElement &xml_ele, const std::string &attribute_name, std::uint16_t default_value)->std::uint16_t;
		static auto attributeUint8(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint8_t;
		static auto attributeUint8(const core::XmlElement &xml_ele, const std::string &attribute_name, std::uint8_t default_value)->std::uint8_t;
		static auto attributeFloat(const core::XmlElement &xml_ele, const std::string &attribute_name)->float;
		static auto attributeFloat(const core::XmlElement &xml_ele, const std::string &attribute_name, float default_value)->float;
		static auto attributeDouble(const core::XmlElement &xml_ele, const std::string &attribute_name)->double;
		static auto attributeDouble(const core::XmlElement &xml_ele, const std::string &attribute_name, double default_value)->double;
		static auto attributeString(const core::XmlElement &xml_ele, const std::string &attribute_name)->std::string;
		static auto attributeString(const core::XmlElement &xml_ele, const std::string &attribute_name, const std::string &default_value)->std::string;
		static auto attributeChar(const core::XmlElement &xml_ele, const std::string &attribute_name)->char;
		static auto attributeChar(const core::XmlElement &xml_ele, const std::string &attribute_name, char default_value)->char;
		template<typename ChildType>
		static auto registerTypeGlobal()->int 
		{ 
			static int count{ 0 };
			TypeInfo::CreateTypeInfo<ChildType>().registerTo(ChildType::Type());
			return ++count;
		}
		template<typename ChildType>
		auto registerType()->void { TypeInfo::CreateTypeInfo<ChildType>().registerTo(ChildType::Type(), *this); }
		auto getTypeInfo(const std::string &type_name)const->const TypeInfo*;
		static auto Type()->const std::string & { static const std::string type("Object"); return std::ref(type); }
		auto virtual type() const->const std::string& { return Type(); }
		auto virtual loadXml(const core::XmlElement &xml_ele)->void;
		auto virtual saveXml(core::XmlElement &xml_ele) const->void;
		auto loadXmlFile(const std::string &filename)->void;
		auto saveXmlFile(const std::string &filename) const->void;
		auto loadXmlDoc(const core::XmlDocument &xml_doc)->void;
		auto saveXmlDoc(core::XmlDocument &xml_doc)const->void;
		auto loadXmlStr(const std::string &xml_str)->void { 
			core::XmlDocument xml_doc; 
			xml_doc.Parse(xml_str.c_str()); 
			loadXmlDoc(xml_doc); };
		auto saveXmlStr(std::string &xml_str)const->void { xml_str = xmlString(); };
		auto xmlString()const->std::string;
		auto name()const->const std::string&;
		auto setName(const std::string& name)->void;
		auto id()const->std::size_t;
		auto root()->Object&;
		auto root()const->const Object& { return const_cast<std::decay_t<decltype(*this)> *>(this)->root(); }
		template<typename AncestorType>
		auto ancestor()->AncestorType* { return father() ? (dynamic_cast<AncestorType*>(father()) ? dynamic_cast<AncestorType*>(father()) : father()->ancestor<AncestorType>()) : nullptr; };
		template<typename AncestorType>
		auto ancestor()const->const AncestorType* { return const_cast<std::decay_t<decltype(*this)> *>(this)->ancestor<AncestorType>(); }
		auto father()->Object*;
		auto father()const->const Object* { return const_cast<std::decay_t<decltype(*this)> *>(this)->father(); }
		auto children()->ImpContainer<Object>&;
		auto children()const->const ImpContainer<Object>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->children(); }
		auto findByName(const std::string &name)const->ImpContainer<Object>::const_iterator { return const_cast<std::decay_t<decltype(*this)> *>(this)->findByName(name); }
		auto findByName(const std::string &name)->ImpContainer<Object>::iterator;
		template<typename T = Object>
		auto findType()const->const T* { return const_cast<Object*>(this)->findType<T>(); };
		template<typename T = Object>
		auto findType()->T* { auto ret = std::find_if(children().begin(), children().end(), [](Object &p) {return dynamic_cast<T*>(&p); }); return ret == children().end() ? nullptr : dynamic_cast<T*>(&*ret); }
		template<typename T = Object>
		auto findType(const std::string &name)const->const T* { return const_cast<Object*>(this)->findType<T>(name); };
		template<typename T = Object>
		auto findType(const std::string &name)->T* { auto ret = std::find_if(children().begin(), children().end(), [&](Object &p) {return dynamic_cast<T*>(&p) && p.name() == name; }); return ret == children().end() ? nullptr : dynamic_cast<T*>(&*ret); }
		template<typename T = Object, typename ...Args>
		auto findOrInsert(const std::string &name, Args&&... args)-> T* { auto p = findType<T>(name); return p ? p : &add<T>(name, std::forward<Args>(args)...); }
		template<typename T = Object, typename ...Args>
		auto findOrInsertType(Args&&... args)-> T* { auto p = findType<T>(); return p ? p : &add<T>(std::forward<Args>(args)...); }
		auto add(Object *obj)->Object &;
		template<typename T, typename ...Args>
		auto add(Args&&... args)->std::enable_if_t<std::is_base_of<Object, T>::value, T>& { return dynamic_cast<T&>(add(new T(std::forward<Args>(args)...))); }

		virtual ~Object();
		explicit Object(const std::string &name = "");
		Object(const Object &);
		Object(Object &&);
		Object& operator=(const Object &);
		Object& operator=(Object &&);
		
	private:
		struct Imp;
		ImpPtr<Imp> imp_;
	};
	template <class T, class Base = Object> class ObjectPool : public Base
	{
	public:
		static_assert(std::is_base_of<Object, Base>::value, "template param \"Base\" of \"ObjectPool\" must be derived class of \"Object\"");

		using value_type = T;
		using reference = T &;
		using const_reference = const T&;
		using pointer = T * ;
		using const_pointer = const T*;
		using difference_type = typename ImpContainer<Object>::difference_type;
		using size_type = typename ImpContainer<Object>::size_type;

		class iterator
		{
		public:
			using difference_type = typename ObjectPool::difference_type;
			using value_type = typename ObjectPool::value_type;
			using reference = typename ObjectPool::reference;
			using pointer = typename ObjectPool::pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const iterator&other)->iterator& = default;
			auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
			auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->iterator& { ++iter_; return *this; }
			auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
			auto operator--()->iterator& { --iter_; return *this; } //optional
			auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->iterator { return iterator(iter_ + size); } //optional
			friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
			auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->iterator { return iterator(iter_ - size); } //optional
			auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->reference { return static_cast<reference>(iter_.operator*()); }
			auto operator->() const->pointer { return static_cast<pointer>(iter_.operator->()); }
			auto operator[](size_type size) const->reference { return iter_.operator[](size); } //optional

			~iterator() = default;
			iterator() = default;
			iterator(const iterator& other) = default;
			iterator(typename ImpContainer<Object>::iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename ImpContainer<Object>::iterator iter_;
			friend class ObjectPool::const_iterator;
			friend class ObjectPool;
		};
		class const_iterator
		{
		public:
			using difference_type = typename ObjectPool::difference_type;
			using value_type = typename ObjectPool::value_type;
			using const_reference = typename ObjectPool::const_reference;
			using const_pointer = typename ObjectPool::const_pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const const_iterator&)->const_iterator& = default;
			auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
			auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->const_iterator& { ++iter_; return *this; }
			auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
			auto operator--()->const_iterator& { --iter_; return *this; } //optional
			auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); } //optional
			friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); } //optional
			auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); } //optional
			auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->const_reference { return static_cast<const_reference>(iter_.operator*()); }
			auto operator->() const->const_pointer { return static_cast<const_pointer>(iter_.operator->()); }
			auto operator[](size_type size) const->const_reference { return iter_.operator[](size); } //optional

			~const_iterator() = default;
			const_iterator() = default;
			const_iterator(const const_iterator&) = default;
			const_iterator(const iterator& other) :iter_(other.iter_) {}
			const_iterator(typename ImpContainer<Object>::const_iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename ImpContainer<Object>::const_iterator iter_;
		};
		using reverse_iterator = std::reverse_iterator<iterator>; //optional
		using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

		auto size()const->size_type { return Base::children().size(); }
		auto max_size()->size_type { return Base::children().max_size(); }
		auto empty()->bool { return Base::children().empty(); }

		auto begin()->iterator { return Base::children().begin(); }
		auto begin()const->const_iterator { return Base::children().begin(); }
		auto cbegin() const->const_iterator { return Base::children().cbegin(); }
		auto end()->iterator { return Base::children().end(); }
		auto end()const->const_iterator { return Base::children().end(); }
		auto cend() const->const_iterator { return Base::children().cend(); }
		auto rbegin()->reverse_iterator { return Base::children().rbegin(); } //optional
		auto rbegin() const->const_reverse_iterator { return Base::children().rbegin(); }; //optional
		auto crbegin() const->const_reverse_iterator { return Base::children().crbegin(); }; //optional
		auto rend()->reverse_iterator { return Base::children().rend(); } //optional
		auto rend() const->const_reverse_iterator { return Base::children().rend(); } //optional
		auto crend() const->const_reverse_iterator { return Base::children().crend(); } //optional
		auto front()->reference { return *begin(); } //optional
		auto front() const->const_reference { return *begin(); } //optional
		auto back()->reference { return *(end() - 1); } //optional
		auto back() const->const_reference { return *(end() - 1); } //optional
		auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(Base::children().at(id)); }
		auto at(std::size_t id)->reference { return static_cast<reference>(Base::children().at(id)); }
		auto operator[](size_type size)->reference { return static_cast<reference>(Base::children().operator[](size)); } //optional
		auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(Base::children().operator[](size)); } //optional
		auto atNrt(std::size_t id) const->const_reference { return dynamic_cast<const_reference>(Base::children().at(id)); }
		auto atNrt(std::size_t id)->reference { return dynamic_cast<reference>(Base::children().at(id)); }

		auto pop_back()->void { Base::children().pop_back(); } //optional
		auto erase(iterator iter)->iterator { return Base::children().erase(iter.iter_); } //optional
		auto erase(iterator begin_iter, iterator end_iter)->iterator { return Base::children().erase(begin_iter.iter_, end_iter.iter_); } //optional
		auto clear()->void { Base::children().clear(); } //optional

	public:
		static auto Type()->const std::string & {
			static const std::string type{ (&Type == &T::Type ? std::string("Noname") : T::Type()) + "Pool" + (&Type == &Base::Type ? std::string("Noname") : Base::Type()) };
			return type;
		}
		auto virtual type()const->const std::string & override { return Type(); }
		auto findByName(const std::string &name)const->const_iterator { return Base::findByName(name); }
		auto findByName(const std::string &name)->iterator { return Base::findByName(name); }

		auto add(T *obj)->T & { return dynamic_cast<T&>(Object::add(obj)); }
		template<typename TT, typename ...Args>
		auto add(Args&&... args)->std::enable_if_t<std::is_base_of<T, TT>::value, TT>& { return dynamic_cast<TT&>(add(new TT(std::forward<Args>(args)...))); }
		template<typename ...Args>
		auto addChild(Args&&... args)->T& { return dynamic_cast<T&>(add(new T(std::forward<Args>(args)...))); }

		virtual ~ObjectPool() = default;
		explicit ObjectPool(const std::string &name = "object_pool") :Base(name) {}
		ObjectPool(const ObjectPool &) = default;
		ObjectPool(ObjectPool &&) = default;
		ObjectPool& operator=(const ObjectPool &) = default;
		ObjectPool& operator=(ObjectPool &&) = default;


	private:
		//static inline int register_count_ = aris::core::Object::registerTypeGlobal<ObjectPool>();

		friend class Object;
		friend class Root;
	};
	template <class T, class Pool> class SubRefPool
	{
	public:
		using value_type = T;
		using reference = T & ;
		using const_reference = const T&;
		using pointer = T * ;
		using const_pointer = const T*;
		using difference_type = std::size_t;
		using size_type = std::size_t;
		class const_iterator;

		class iterator
		{
		public:
			using difference_type = typename SubRefPool::difference_type;
			using value_type = typename SubRefPool::value_type;
			using reference = typename SubRefPool::reference;
			using pointer = typename SubRefPool::pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const iterator&other)->iterator& = default;
			auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
			auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->iterator& { ++iter_; return *this; }
			auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
			auto operator--()->iterator& { --iter_; return *this; } //optional
			auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->iterator { return iterator(iter_ + size); } //optional
			friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
			auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->iterator { return iterator(iter_ - size); } //optional
			auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->reference { return std::ref(**iter_); }
			auto operator->() const->pointer { return *iter_; }
			auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

			~iterator() = default;
			iterator() = default;
			iterator(const iterator& other) = default;
			iterator(typename std::vector<T*>::iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename std::vector<T*>::iterator iter_;
			friend class SubRefPool::const_iterator;
		};
		class const_iterator
		{
		public:
			using difference_type = typename SubRefPool::difference_type;
			using value_type = typename SubRefPool::value_type;
			using const_reference = typename SubRefPool::const_reference;
			using const_pointer = typename SubRefPool::const_pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const const_iterator&)->const_iterator& = default;
			auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
			auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->const_iterator& { ++iter_; return *this; }
			auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
			auto operator--()->const_iterator& { --iter_; return *this; } //optional
			auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); } //optional
			friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); } //optional
			auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); } //optional
			auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->const_reference { return **iter_; }
			auto operator->() const->const_pointer { return *iter_; }
			auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

			~const_iterator() = default;
			const_iterator() = default;
			const_iterator(const const_iterator&) = default;
			const_iterator(const iterator& other) :iter_(other.iter_) {}
			const_iterator(typename std::vector<T*>::const_iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename std::vector<T*>::const_iterator iter_;
		};
		using reverse_iterator = std::reverse_iterator<iterator>; //optional
		using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

		auto size()const->size_type { return container_.size(); }
		auto max_size()->size_type { return container_.max_size(); }
		auto empty()->bool { return container_.empty(); }

		auto begin()->iterator { return container_.begin(); }
		auto begin()const->const_iterator { return container_.begin(); }
		auto cbegin() const->const_iterator { return container_.cbegin(); }
		auto end()->iterator { return container_.end(); }
		auto end()const->const_iterator { return container_.end(); }
		auto cend() const->const_iterator { return container_.cend(); }
		auto rbegin()->reverse_iterator { return container_.rbegin(); } //optional
		auto rbegin() const->const_reverse_iterator { return container_.rbegin(); } //optional
		auto crbegin() const->const_reverse_iterator { return container_.crbegin(); } //optional
		auto rend()->reverse_iterator { return container_.rend(); } //optional
		auto rend() const->const_reverse_iterator { return container_.rend(); } //optional
		auto crend() const->const_reverse_iterator { return container_.crend(); } //optional
		auto front()->reference { return *begin(); } //optional
		auto front() const->const_reference { return *begin(); } //optional
		auto back()->reference { return *(end() - 1); } //optional
		auto back() const->const_reference { return *(end() - 1); } //optional
		auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(*container_.at(id)); }
		auto at(std::size_t id)->reference { return static_cast<reference>(*container_.at(id)); }
		auto operator[](size_type size)->reference { return static_cast<reference>(*container_.operator[](size)); } //optional
		auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(*container_.operator[](size)); } //optional

	public:
		auto findByName(const std::string &name)const->const_iterator { return std::find_if(begin(), end(), [&name, this](T &p) {return (p.name() == name); }); }
		auto findByName(const std::string &name)->iterator { return std::find_if(begin(), end(), [&name, this](T &p) {return (p.name() == name); }); }
		auto update() 
		{
			container_.clear();
			for (auto &obj : *pool_)
			{
				if (auto child = dynamic_cast<T*>(&obj))
				{
					container_.push_back(child);
				}
			}
		}
		auto swap(SubRefPool& other)->void { return container_.swap(other.container_); }

		SubRefPool(Pool* target) { pool_ = target; }

	private:
		std::vector<T*> container_;
		Pool* pool_;
	};
	template <class T, class Pool> class ChildRefPool
	{
	public:
		using value_type = T;
		using reference = T & ;
		using const_reference = const T&;
		using pointer = T * ;
		using const_pointer = const T*;
		using difference_type = std::size_t;
		using size_type = std::size_t;
		class const_iterator;

		class iterator
		{
		public:
			using difference_type = typename ChildRefPool::difference_type;
			using value_type = typename ChildRefPool::value_type;
			using reference = typename ChildRefPool::reference;
			using pointer = typename ChildRefPool::pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const iterator&other)->iterator& = default;
			auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
			auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->iterator& { ++iter_; return *this; }
			auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
			auto operator--()->iterator& { --iter_; return *this; } //optional
			auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->iterator { return iterator(iter_ + size); } //optional
			friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
			auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->iterator { return iterator(iter_ - size); } //optional
			auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->reference { 
				auto& s = *iter_;
				return dynamic_cast<T&>(*iter_); 
			}
			auto operator->() const->pointer { return dynamic_cast<T*>(&*iter_); }
			auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

			~iterator() = default;
			iterator() = default;
			iterator(const iterator& other) = default;
			iterator(typename Pool::iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename Pool::iterator iter_;
			friend class ChildRefPool::const_iterator;
		};
		class const_iterator
		{
		public:
			using difference_type = typename ChildRefPool::difference_type;
			using value_type = typename ChildRefPool::value_type;
			using const_reference = typename ChildRefPool::const_reference;
			using const_pointer = typename ChildRefPool::const_pointer;
			using iterator_category = std::random_access_iterator_tag; //or another tag

			auto operator=(const const_iterator&)->const_iterator& = default;
			auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
			auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
			auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
			auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
			auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
			auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

			auto operator++()->const_iterator& { ++iter_; return *this; }
			auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
			auto operator--()->const_iterator& { --iter_; return *this; } //optional
			auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
			auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
			auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); } //optional
			friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); } //optional
			auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
			auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); } //optional
			auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

			auto operator*() const->const_reference { return **iter_; }
			auto operator->() const->const_pointer { return *iter_; }
			auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

			~const_iterator() = default;
			const_iterator() = default;
			const_iterator(const const_iterator&) = default;
			const_iterator(const iterator& other) :iter_(other.iter_) {}
			const_iterator(typename Pool::const_iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename Pool::const_iterator iter_;
		};
		using reverse_iterator = std::reverse_iterator<iterator>; //optional
		using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

		auto size()const->size_type { return pool_->size(); }
		auto max_size()->size_type { return pool_->max_size(); }
		auto empty()->bool { return pool_->empty(); }

		auto begin()->iterator { return pool_->begin();	}
		auto begin()const->const_iterator { return pool_->begin(); }
		auto cbegin() const->const_iterator { return pool_->cbegin(); }
		auto end()->iterator { return pool_->end(); }
		auto end()const->const_iterator { return pool_->end(); }
		auto cend() const->const_iterator { return pool_->cend(); }
		auto rbegin()->reverse_iterator { return pool_->rbegin(); } //optional
		auto rbegin() const->const_reverse_iterator { return pool_->rbegin(); } //optional
		auto crbegin() const->const_reverse_iterator { return pool_->crbegin(); } //optional
		auto rend()->reverse_iterator { return pool_->rend(); } //optional
		auto rend() const->const_reverse_iterator { return pool_->rend(); } //optional
		auto crend() const->const_reverse_iterator { return pool_->crend(); } //optional
		auto front()->reference { return *begin(); } //optional
		auto front() const->const_reference { return *begin(); } //optional
		auto back()->reference { return *(end() - 1); } //optional
		auto back() const->const_reference { return *(end() - 1); } //optional
		auto at(std::size_t id) const->const_reference { return dynamic_cast<const_reference>(pool_->at(id)); }
		auto at(std::size_t id)->reference { return dynamic_cast<reference>(pool_->at(id)); }
		auto operator[](size_type size)->reference { return dynamic_cast<reference>(pool_->operator[](size)); } //optional
		auto operator[](size_type size) const->const_reference { return dynamic_cast<const_reference>(pool_->operator[](size)); } //optional
		
		auto pop_back()->void { pool_->pop_back(); } //optional
		auto erase(iterator iter)->iterator { return pool_->erase(iter.iter_); } //optional
		auto erase(iterator begin_iter, iterator end_iter)->iterator { return pool_->erase(begin_iter.iter_, end_iter.iter_); } //optional
		auto clear()->void { pool_->clear(); } //optional

	public:
		auto findByName(const std::string &name)const->const_iterator { return std::find_if(begin(), end(), [&name, this](T &p) {return (p.name() == name); }); }
		auto findByName(const std::string &name)->iterator { return std::find_if(begin(), end(), [&name, this](T &p) {return (p.name() == name); }); }
		auto add(T *obj)->T & { return dynamic_cast<T&>(pool_->add(obj)); }
		template<typename TT, typename ...Args>
		auto add(Args&&... args)->std::enable_if_t<std::is_base_of<T, TT>::value, TT>& { return dynamic_cast<TT&>(pool_->add(new TT(std::forward<Args>(args)...))); }
		template<typename ...Args>
		auto addChild(Args&&... args)->T& { return dynamic_cast<T&>(pool_->add(new T(std::forward<Args>(args)...))); }
		
		ChildRefPool(Pool* target) { pool_ = target; }

	private:
		Pool *pool_;
	};

#define CODEIT_DEFINE_TYPE_NAME(type_name) \
	static auto Type()->const std::string & { \
		static const std::string type(type_name); \
		return std::ref(type); \
	} \
	auto virtual type() const->const std::string& override { return Type(); }

#define CODEIT_REGISTER_TYPE(type_name) \
	CODEIT_DEFINE_TYPE_NAME(#type_name) \
	static inline int register_count_ = core::Object::registerTypeGlobal<type_name>();
	
#define CODEIT_DECLARE_BIG_FOUR(type_name) \
	type_name(const type_name &other); \
	type_name(type_name &&other); \
	type_name& operator=(const type_name &other); \
	type_name& operator=(type_name &&other);

#define CODEIT_DEFINE_BIG_FOUR(type_name) \
	type_name(const type_name &other) = default; \
	type_name(type_name &&other) = default; \
	type_name& operator=(const type_name &other) = default; \
	type_name& operator=(type_name &&other) = default;

#define CODEIT_DEFINE_BIG_FOUR_CPP(type_name) \
	type_name::type_name(const type_name &other) = default; \
	type_name::type_name(type_name &&other) = default; \
	type_name& type_name::operator=(const type_name &other) = default; \
	type_name& type_name::operator=(type_name &&other) = default;

#define CODEIT_DELETE_BIG_FOUR(type_name) \
	type_name(const type_name &other) = delete; \
	type_name(type_name &&other) = delete; \
	type_name& operator=(const type_name &other) = delete; \
	type_name& operator=(type_name &&other) = delete;

	template<typename T>
	static auto allocMem(Size &mem_pool_size, T* &pointer, Size size)->void
	{
		*reinterpret_cast<Size*>(&pointer) = mem_pool_size;
		mem_pool_size += sizeof(T) * size;
	}
	template<typename T>
	static auto getMem(char *mem_pool, T* &pointer)->T*
	{
		return reinterpret_cast<T*>(mem_pool + *reinterpret_cast<Size*>(&pointer));
	}

	///
	///  @}
	///
}

#endif
