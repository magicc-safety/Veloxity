// /**
// ******************************************************************************
// * File     : hlist.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
// ============================================================================
// HList Definitions - For compile time decisions on sensors
// ============================================================================
use crate::params2::Params;
use crate::sensorprocessors::CalibrationFlags;
use core::marker::PhantomData;

// Represents the end of a heterogeneous list. It's an empty list.
#[derive(Debug, Default, Clone, Copy)]
pub struct HNil;

// Represents one "link" in a heterogeneous list.
// H (Head): The type of the data stored in this link.
// T (Tail): The type of the rest of the list.
#[derive(Debug, Default, Clone, Copy)]
pub struct HCons<H, T>(pub H, pub T);

// A marker trait to label both types as being a HList
pub trait HList {}

// The first line says HNil is a valid HList
// The second line says HCons is a valid HList if and only if it's tail T is a valid HList
impl HList for HNil {}
impl<H, T: HList> HList for HCons<H, T> {}

// ============================================================================
// HList Operation Mechanics - processing logic differs for each sensor in list
// ============================================================================

// A generic trait for a function-like object. We use this instead of the
// standard `Fn` traits because we need an associated `Output` type.
pub trait Func<Arg> {
    type Output;
    fn call(&mut self, arg: Arg, flags: &mut CalibrationFlags, params: &mut Params) -> Self::Output;
}

// A marker trait for a "Polymorphic Function" (an HList of `Func`s).
// Requires `Copy` because we will pass the processor list by value, and `Default`
// so we can create instances of them easily.
// ---------------------------------------------
// Basically defining an HList of function objects that can each
// 1) be copied
// 2) be default-constructed
// 3) have consistent structure (ending in HNil)
// ---------------------------------------------
// copy trait included here so we can copy a full pipeline of functions if desired
pub trait PolyFunc: Copy + Default {}
impl PolyFunc for HNil {}
impl<F: Copy + Default, T: PolyFunc> PolyFunc for HCons<F, T> {}

// ============================================================================================
// HList Mapping Functionality - connecting HList (the mapped) to funciton HList (the mapper)
// ============================================================================================

// Example HMappable in action:
// -------------------------------
// Data HList: HCons(sensor1, HCons(sensor2, HNil))
// Func HList: HCons(process1, HCons(process2, HNil))
// Result:     HCons(process1(&mut sensor1), HCons(process2(&mut sensor2), HNil))

pub trait HMappable<'a, Mapper: PolyFunc> {
    type Output: HList;
    fn map(
        &'a mut self,
        f: Mapper,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output;
}

// make HNil into something that can be mapped (the mapped) by a mappable (a PolyFunc)
impl<'a, Mapper: PolyFunc> HMappable<'a, Mapper> for HNil {
    type Output = HNil;
    fn map(
        &'a mut self,
        _f: Mapper,
        _flags: &mut CalibrationFlags,
        _params: &mut Params,
    ) -> Self::Output {
        HNil
    }
}

// make the HList mappable (recursive definition)
// F for Function, D for data
// note how HCons is the Mapper
impl<'a, F, FTail, D, DTail> HMappable<'a, HCons<F, FTail>> for HCons<D, DTail>
where
    F: Func<&'a mut D> + Copy + Default, // function is operable with data type D as it's Arg
    FTail: PolyFunc,                     // recursive definition
    D: 'a,                               // obviously
    DTail: HList + HMappable<'a, FTail> + 'a, // DTail should be a HList for recursion, and it needs the
                                              // same lifetime, and it also needs to be mapable via/by a
                                              // mapper (and FTail satisfies that because FTail is of type PolyFunc)
{
    // Output is of type HCons<Head where head is the output of applying F to the head data, Tail:
    // the result of recursively mapping the tail
    type Output = HCons<F::Output, <DTail as HMappable<'a, FTail>>::Output>;

    fn map(
        &'a mut self,
        mut f: HCons<F, FTail>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        // 1) apply head function f.0 to head data self.0
        // 2) recursively map the tail data self.1 with the tail functions f.1
        //    and combine results into a new HCons
        HCons(
            f.0.call(&mut self.0, flags, params),
            self.1.map(f.1, flags, params),
        )
    }
}

// ============================================================================================
// Plucker and Sculpter Functionality - connecting HList (the mapped) to funciton HList (the mapper)
// ============================================================================================

pub enum Here {}
pub struct There<I>(PhantomData<I>);

// Solve the following problem: find and extract a single, specific type from an HList and get back
// both that item and the rest of the list separately. Equivalent to a list.pop(idx) but verified
// via compiler

// Plucker is generic of T (target) and Index (Here or There)
pub trait Plucker<Target, Index> {
    type Remainder: HList;

    // split into the two pieces we need
    fn pluck(self) -> (Target, Self::Remainder);
}

// impl<T, Tail: HList> define implementation generic over T and with any type Tail that's a valid HList
// Plucker<T, Here> specifies WHICH plucker we're implementing. It's the version where the target is T and the Index is Here
// for HCons<T, Tail> says implementing FOR an HCons struct... specifically it's for HCons who's head is of type T (same T as our target) and who's tail is of type Tail
// --------
// so in english: implementing plucking an item of type T where the index is Here aka front of the
// list, and this rule only applies to an HCons who's head is already the type T we're looking for
// and who's tail is another HList
impl<T, Tail: HList> Plucker<T, Here> for HCons<T, Tail> {
    // if we pluck the head, the remainder is logically the tail
    type Remainder = Tail;

    // take ownership of the "HCons" it's called on. It returns a tuple containing the Target T,
    // and the tail...
    fn pluck(self) -> (T, Self::Remainder) {
        (self.0, self.1) // HCons is a tuple struct, so this type being a tuple matches perfectly. It just deconstructs the HCons into it's two constituent pieces
    }
}

// impl<H, ...> says defining a generic implementation over the Head = of the current list link, Tail = the type of Tail of the current list link, T = Target we're ultimately looking for, TailIdx = the "inner" index that we'll use for the recursive search on the "Tail"
// Plucker<T, There<TailIdx>> says implementing the Plucker trait for the case where Index is "There<...>" aka not "Here"
// --------
// Item we're looking for is NOT at the head of the index...
// --------
// This is the rule for plucking a target T from an HCons when the index tells us the target is somewhere further down the list There<...>
impl<H, Tail, T, TailIdx> Plucker<T, There<TailIdx>> for HCons<H, Tail>
where
    Tail: HList + Plucker<T, TailIdx>, // Implementation is only valid if the "Tail" is also pluckable for the same target T using the inner index TailIdx
                                       // AKA the compiler can guarantee that if we call pluck on the tail, it will find a valid implementation for it (either a recursive one again,
                                       // or the Here base case)
{
    // says the Tail will have it's own Remainder after the recursive pluck is done...
    // the final remainder of the whole list is the original head H re-attached to the front of the
    // tail's remainder
    type Remainder = HCons<H, <Tail as Plucker<T, TailIdx>>::Remainder>;
    fn pluck(self) -> (T, Self::Remainder) {
        // pluck the tail. Because of the where clause, this is valid. eventually will return the
        // target we're interested in, as well as the tail remainder.
        // ------------
        // Onion analogy:
        // 1) You look at the current layer (Hcons). The item isn't There
        // 2) You peel off the outer layer (self.0) and set it aside
        // 3) You hand the rest of the onion (self.1) to a helper and ask them to find the item
        // 4) They return with the item (target) and the rest of the onion with that item removed
        //    (tail_rem)
        // 5) you take the outer layer you set aside (self.0) and put it back around the modified
        //    inner onion (tail_rem). You now have the complete onion reassembled without that
        //    item, AND the item.
        // ------------
        // notice how if it can't find the item in the list, it fails to compile!!! Guarantees compatibility
        let (target, tail_rem) = self.1.pluck();
        (target, HCons(self.0, tail_rem))
    }
}

// Plucker extracts single items from the HList...
// Sculptor uses Plucker repeatedly to build a new HList from a subset of the original
// -----------
// The question this answers is can I build this specific target list (RequiredSensors) by picking
// and choosing items from this source list (ProcessedSensorSet) in a specific order?
// -----------
// this is responsible for compile-time assurance that Board is compatible with Vehicle...
// -----------
// Target: Hlist: this represents the shape of the HList we want to create. For example:
// Hcons<Option<CalibratedImu>, HCons<Option<FilteredBaro>, HNil>> <-- basically the blueprint for the final product
// -----------
// Indicies: HList: an HList of type-level indicies like Here, There<..>. This list provides exact
// instructions for where to find each element of the Target list within the source list.
pub trait Sculptor<Target: HList, Indicies: HList> {
    type Remainder: HList;
    // return a tuple containing two new owned HLists: the Target list that was successfully built,
    // and the Remainder list. Sculpt does consume the HList
    fn sculpt(self) -> (Target, Self::Remainder);
}

// Simplest rule for termination condition for recursion: if asked to sculpt an empty list HNil
// from any source S, the result is an empty list HNil and the remainder is the entire source you
// started with (Sculptor<HNil, HNil> the first is the Target,the second is the Indicies). so it's
// asking what happens when I'm asked to build an empty set from an empty list?
// --------
// for S: this says we're implementing the trait for S, which is any source list
// Sculptor<Target, Indicies> says what is the shape of the list we're trying to build, and what
//                            are the instructions (Indicies) for building it?
impl<S: HList> Sculptor<HNil, HNil> for S {
    type Remainder = S;
    fn sculpt(self) -> (HNil, Self::Remainder) {
        (HNil, self)
    }
}

// Heavy lifting: connect Board's full sensor list (processed or not depends on flags as we'll see
// later) with the Vehicle's specific needs. in order:
// 1) pluck the first item you need.
// 2) Recursively sculpt the rest of the list from the remainder
// 3) Combine the results
// ------------
// For S (any source list that's an HList (see where clause)): we are implementing the trait,
// and the Target List and the Index list are non-empty. There are Target Heads, and Target Tails,
// and Index Heads and Index Tails since they're all HLists
impl<S, THead, TTail, IHead, ITail> Sculptor<HCons<THead, TTail>, HCons<IHead, ITail>> for S
where
    S: HList + Plucker<THead, IHead>, // says it is possible to pluck the first item we need from
    // the source list S using the first instruction IHead. Compiler searches for a valid Plucker
    // implementation
    TTail: HList,
    ITail: HList,
    <S as Plucker<THead, IHead>>::Remainder: Sculptor<TTail, ITail>, // recursive check: says after
                                                                     // we pluck that first item,
                                                                     // take the REMAINDER of the source list. Then is it possible to recursively skulpt the rest of
                                                                     // the target list (TTail) from that remainder using the rest of the instructions (ITail)?
{
    type Remainder = <<S as Plucker<THead, IHead>>::Remainder as Sculptor<TTail, ITail>>::Remainder;
    fn sculpt(self) -> (HCons<THead, TTail>, Self::Remainder) {
        // generates code to call the correct "pluck" implementation. Givues us our target list
        // (head), and the sources list with pieces removed (rem1).
        let (head, rem1) = self.pluck();
        // has also proven the recursive is true... generates code to call sculpt for the rest of
        // the lists. This gives us our fully-formed tail of our target list (tail) and the final
        // remainder of the source list (rem2)
        let (tail, rem2) = rem1.sculpt();
        // Finally, generates code to construct a new HCons cell from the head it got in step 1 and
        // the tail in step 2. Returned along with final reminder.
        (HCons(head, tail), rem2)
    }
}

/// Trait to get an immutable reference to an element by its index.
pub trait HListGet<Target, Index> {
    fn get(&self) -> &Target;
}

/// Base case: The target is at the Head (Here).
impl<T, Tail: HList> HListGet<T, Here> for HCons<T, Tail> {
    fn get(&self) -> &T {
        &self.0 // Return a reference to the head
    }
}

/// Recursive case: The target is in the Tail (There).
impl<H, Tail, T, TailIdx> HListGet<T, There<TailIdx>> for HCons<H, Tail>
where
    Tail: HList + HListGet<T, TailIdx>, // We can `get` the target from the tail
{
    fn get(&self) -> &T {
        self.1.get() // Recurse on the tail
    }
}

/// It uses `$crate::packets` to ensure paths are always valid. Change this if the HList definition
/// moves
#[macro_export]
macro_rules! hlist_type {
    // Base case: empty list becomes HNil
    () => {
        $crate::hlist::HNil
    };

    // Recursive case: single element followed by rest
    ($head:ty $(, $tail:ty)*) => {
        $crate::hlist::HCons<$head, hlist_type![$($tail),*]>
    };
}
